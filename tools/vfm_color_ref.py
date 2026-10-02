# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Floating-point reference decoder for the colour of FM video (analog FPV, PAL/NTSC composite).

Used offline on raw captures of the ESP32-C5 (PARLIO PCAP bytes at 40 MS/s 4+4 bit, or 10-bit CAPT20 words at
20 MS/s) and on simulated signals, to see what colour information the samples contain before anything is put on
the device (docs/espsoup-c5.md, section "Colour (VFMCOLOR)").

Method (all floating point, no attempt at speed):
  phase of every sample -> unwrapped phase -> instantaneous frequency f[n] (Hz) at the sample rate
  luma: f low-passed by a boxcar of one subcarrier period (notch at fsc), sync edges from the luma
  chroma: f[n] * exp(-j 2 pi fsc n / fs) with n the ABSOLUTE sample index (one phase-continuous oscillator for the
          whole capture), averaged over the burst window and over nseg segments of the active line
  reference: per line the burst phasors of the neighbouring lines (PAL: their mean direction is the "90 degree"
          axis, the +-45 degree swing gives the V switch of the line; NTSC: burst = 180 degrees = -U)
  saturation: in volts from the luma calibration (sync tip .. blanking = 0.3 V PAL / 0.286 V NTSC) and, as a second
          scale, relative to the burst amplitude (nominal 0.15 / 0.143 V).

  python vfm_color_ref.py sim  <file.iq int16 40 MS/s> <out_prefix> [--src parlio|dump20|float] [--cnr dB] [--off MHz] [--std PAL|NTSC]
  python vfm_color_ref.py pcap <capture.npz> <out_prefix>      (captures written by vfm_color_capture.py)
"""
import argparse, json, math, os, sys
import numpy as np

FSC = {"PAL": 4433618.75, "NTSC": 3579545.45}
LINE_T = {"PAL": 64e-6, "NTSC": 1 / 15734.264}
LINES = {"PAL": 625, "NTSC": 525}
BURST = {"PAL": (5.6e-6, 2.25e-6), "NTSC": (5.3e-6, 2.5e-6)}            # start after the sync edge, length
ACT = {"PAL": (10.5e-6, 52e-6), "NTSC": (9.4e-6, 52.6e-6)}
SYNC_V = {"PAL": 0.3, "NTSC": 0.286}
BURST_V = {"PAL": 0.15, "NTSC": 0.143}
WHITE_V = {"PAL": 0.7, "NTSC": 0.714 - 0.0536}                             # white - black (NTSC: 7.5 IRE setup)
SETUP_V = {"PAL": 0.0, "NTSC": 0.0536}


# ------------------------------------------------------------------ inputs
def nibbles(b):
    lo = (b & 15).astype(np.int16); hi = (b >> 4).astype(np.int16)
    lo[lo >= 8] -= 16; hi[hi >= 8] -= 16
    return lo, hi


def phase_parlio(b, step=1):
    """PARLIO bytes (low nibble I, high nibble Q, device orientation) -> phase (rad) of the cell centres, every step-th byte."""
    lo, hi = nibbles(np.asarray(b, np.uint8)[::step])
    return np.arctan2(hi + 0.5, lo + 0.5)


def phase_words(w):
    """10-bit dump words (I bits 0-9, Q bits 10-19, device orientation) -> phase"""
    w = np.asarray(w, np.uint32)
    i = (w & 1023).astype(np.int32); q = ((w >> 10) & 1023).astype(np.int32)
    i[i >= 512] -= 1024; q[q >= 512] -= 1024
    return np.arctan2(q + 0.5, i + 0.5)


def to_parlio(iq, amp=6.0, cnr_db=None, rng=None, lsb_amp=None):
    """Complex baseband (|x| ~ 1) -> what the PARLIO path delivers: 10-bit ADC, 4-bit window [lsb+3:lsb] (wrapping),
    amplitude `amp` in 4-bit LSB units (PLSB AUTO aims at ~6.5).  Returns uint8 bytes."""
    rng = rng or np.random.default_rng(1)
    x = iq * amp * 16.0                                    # lsb 4: 10-bit units = 16 x 4-bit units
    if cnr_db is not None:
        p = np.mean(np.abs(x) ** 2) / 10 ** (cnr_db / 10)
        x = x + (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x))) * math.sqrt(p / 2)
    i = np.clip(np.floor(x.real), -512, 511).astype(np.int32); q = np.clip(np.floor(x.imag), -512, 511).astype(np.int32)
    i4 = (i >> 4) & 15; q4 = (q >> 4) & 15                 # window [7:4], two's complement wrap like the hardware
    return (i4 | (q4 << 4)).astype(np.uint8)


def to_dump20(iq, amp=100.0, cnr_db=None, rng=None):
    """Complex baseband at 40 MS/s -> 10-bit dump words at 20 MS/s (every 2nd sample: the dump drops samples)."""
    rng = rng or np.random.default_rng(1)
    x = iq[::2] * amp
    if cnr_db is not None:
        p = np.mean(np.abs(x) ** 2) / 10 ** (cnr_db / 10)
        # noise of the 40 MS/s band, decimated without filter: same power per sample
        x = x + (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x))) * math.sqrt(p / 2)
    i = np.clip(np.floor(x.real), -512, 511).astype(np.int32); q = np.clip(np.floor(x.imag), -512, 511).astype(np.int32)
    return ((i & 1023) | ((q & 1023) << 10)).astype(np.uint32)


# ------------------------------------------------------------------ decoder
def inst_freq(phi, fs):
    """wrapped phase differences (lag 1) -> Hz, same length (first sample repeated)"""
    d = np.angle(np.exp(1j * np.diff(phi)))
    f = d * fs / (2 * np.pi)
    return np.concatenate([[f[0]], f])


def boxcar(x, n):
    if n <= 1:
        return x.copy()
    c = np.cumsum(np.concatenate([[0], x]))
    y = (c[n:] - c[:-n]) / n
    pad = n // 2
    return np.concatenate([np.full(pad, y[0]), y, np.full(len(x) - len(y) - pad, y[-1])])


def find_syncs(luma, fs, T):
    """Polarity + sync leading edges (float sample positions) from the luma signal."""
    best = None
    for pol in (1, -1):
        z = pol * luma
        q3, q12 = np.percentile(z, 3), np.percentile(z, 12)
        if q12 - q3 <= 0:
            continue
        thr = (q3 + q12) / 2
        below = z < thr
        d = np.diff(below.astype(np.int8))
        starts = np.nonzero(d == 1)[0] + 1; ends = np.nonzero(d == -1)[0] + 1
        if len(starts) < 3:
            continue
        ends = ends[ends > starts[0]]
        m = min(len(starts), len(ends)); starts, ends = starts[:m], ends[:m]
        L = (ends - starts) / fs
        ok = (L > 3.4e-6) & (L < 6.0e-6)
        s = starts[ok]
        # interpolated crossing
        e = s - 1 + (z[s - 1] - thr) / np.maximum(z[s - 1] - z[s], 1e-9)
        sp = np.diff(e) / fs
        good = np.sum(np.abs(sp - T) < 0.5e-6)
        if best is None or good > best[0]:
            best = (good, pol, e, thr)
    if best is None:
        return 1, np.array([]), 0
    return best[1], best[2], best[3]


def decode(phi, fs, std="PAL", nseg=32, n0=0, line_of=None, px=256):
    """phi: phase at rate fs (rad), n0 = absolute index of sample 0.  Returns a dict with per-line records."""
    fsc, T = FSC[std], LINE_T[std]
    f = inst_freq(phi, fs)
    nb = max(1, int(round(fs / fsc)))
    luma = boxcar(boxcar(f, nb), max(1, int(round(fs * 0.1e-6))))
    pol, edges, thr = find_syncs(luma, fs, T)
    z = pol * f; zl = pol * luma
    n = np.arange(len(f)) + n0
    osc = np.exp(-2j * np.pi * fsc * n / fs)
    c = (z - pol * boxcar(f, nb)) * osc                    # luma removed (one-period boxcar = notch at fsc), else DC leaks
    cs = np.concatenate([[0], np.cumsum(c)]); ls = np.concatenate([[0], np.cumsum(zl)])

    def cmean(a, b):
        a = int(round(a)); b = int(round(b))
        return (cs[b] - cs[a]) / max(1, b - a)

    def lmean(a, b):
        a = int(round(a)); b = int(round(b))
        return (ls[b] - ls[a]) / max(1, b - a)

    b0, bl = BURST[std]; a0, al = ACT[std]
    recs = []
    for k, e in enumerate(edges):
        if e + (a0 + al) * fs + 2 >= len(f) or e < 2:
            continue
        tip = lmean(e + 1.0e-6 * fs, e + 4.0e-6 * fs)
        blank = lmean(e + 8.2e-6 * fs, e + 10.2e-6 * fs) if std == "PAL" else lmean(e + 8.0e-6 * fs, e + 9.2e-6 * fs)
        hz_per_v = (blank - tip) / SYNC_V[std]
        zb = cmean(e + (b0 + 0.15 * bl) * fs, e + (b0 + 0.85 * bl) * fs)
        seg = [cmean(e + (a0 + al * j / nseg) * fs, e + (a0 + al * (j + 1) / nseg) * fs) for j in range(nseg)]
        # luma pixels (fraction of white)
        xs = e + (a0 + al * (np.arange(px) + 0.5) / px) * fs
        Y = (np.interp(xs, np.arange(len(zl)), zl) - blank) / hz_per_v
        Y = (Y - SETUP_V[std]) / WHITE_V[std]
        recs.append(dict(edge=e, t=(e + n0) / fs, tip=tip, blank=blank, hz_per_v=hz_per_v, zb=zb, seg=np.array(seg), Y=Y))
    return dict(pol=pol, recs=recs, std=std, fs=fs, nseg=nseg)


def chroma_uv(dec, nref=4):
    """Burst reference, V switch and U/V (fractions of white, gen_tv.mjs scale) per line."""
    std, recs = dec["std"], dec["recs"]
    zb = np.array([r["zb"] for r in recs])
    ub = zb / np.maximum(np.abs(zb), 1e-9)
    out = []
    for k, r in enumerate(recs):
        lo, hi = max(0, k - nref), min(len(recs), k + nref + 1)
        w = np.ones(hi - lo); w[0] = w[-1] = 0.5             # even weight of both burst phases (PAL +-45 deg)
        R = (ub[lo:hi] * w).sum()
        R = R / max(abs(R), 1e-9)
        rot = np.exp(1j * (np.pi / 2 - np.angle(R)))
        zbr = r["zb"] * rot
        vsw = (1 if zbr.real >= 0 else -1) if std == "PAL" else 1
        s = r["seg"] * rot
        # true z = (s*V - j*U)/2 in Hz -> volts via hz_per_v -> fraction of white
        hv = r["hz_per_v"] if r["hz_per_v"] else 1
        U = -2 * s.imag / hv / WHITE_V[std]
        V = 2 * s.real * vsw / hv / WHITE_V[std]
        bamp = 2 * abs(r["zb"]) / hv                       # volts (central burst part, sin^2 envelope included)
        out.append(dict(U=U, V=V, vsw=vsw, burst_v=bamp, burst_ang=float(np.degrees(np.angle(zbr)))))
    return out


def yuv_rgb(Y, U, V):
    B = Y + U / 0.493
    R = Y + V / 0.877
    G = (Y - 0.299 * R - 0.114 * B) / 0.587
    return np.clip(np.stack([R, G, B], -1), 0, 1)


def upsample(seg, px):
    """nseg chroma values -> px (linear between segment centres)"""
    n = len(seg)
    xc = (np.arange(n) + 0.5) / n
    return np.interp((np.arange(px) + 0.5) / px, xc, seg)


# ------------------------------------------------------------------ sim helpers
BARS = [[.75, .75, .75], [.75, .75, 0], [0, .75, .75], [0, .75, 0], [.75, 0, .75], [.75, 0, 0], [0, 0, .75], [0, 0, 0]]
BAR_NAMES = ["white", "yellow", "cyan", "green", "magenta", "red", "blue", "black"]


def bar_truth():
    out = []
    for r, g, b in BARS:
        y = 0.299 * r + 0.587 * g + 0.114 * b
        out.append((y, 0.493 * (b - y), 0.877 * (r - y)))
    return out


def bar_stats(lines_uv, nseg):
    """U/V statistics per colour bar over the given lines (segments fully inside one bar, 3 % .. 97 % of the line)."""
    tr = bar_truth(); res = []
    for k in range(8):
        lo, hi = 0.01 + (k / 8) * 0.96, 0.01 + ((k + 1) / 8) * 0.96
        if k == 7:
            hi = 0.97
        segs = [j for j in range(nseg) if (j / nseg) >= lo + 0.01 and ((j + 1) / nseg) <= hi - 0.01]
        if not segs:
            continue
        U = np.array([l["U"][segs] for l in lines_uv]).ravel(); V = np.array([l["V"][segs] for l in lines_uv]).ravel()
        mu, mv = U.mean(), V.mean(); sd = math.sqrt((U.var() + V.var()) / 2)
        amp = math.hypot(tr[k][1], tr[k][2])
        res.append(dict(bar=BAR_NAMES[k], U=round(mu, 3), V=round(mv, 3), U_true=round(tr[k][1], 3), V_true=round(tr[k][2], 3),
                        noise=round(sd, 4), snr_db=round(20 * math.log10(amp / sd), 1) if amp > 0 and sd > 0 else None,
                        hue_err_deg=round(math.degrees(math.atan2(mv, mu) - math.atan2(tr[k][2], tr[k][1])), 1) if amp > 0 else None,
                        sat_ratio=round(math.hypot(mu, mv) / amp, 3) if amp > 0 else None))
    return res


def save_png(path, rgb, scale=(2, 1)):
    from PIL import Image
    im = Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8))
    im = im.resize((im.width * scale[0], im.height * scale[1]), Image.NEAREST)
    im.save(path)


def frame_image(dec, uv, std, px, line_number):
    """Assemble an interlaced frame (rows = active lines) from records with known line numbers."""
    f1 = (23, 310) if std == "PAL" else (22, 261)
    f2 = (336, 623) if std == "PAL" else (285, 524)
    rows = 2 * (f1[1] - f1[0] + 1)
    img = np.zeros((rows, px, 3)); luma = np.zeros((rows, px)); have = np.zeros(rows, bool)
    for r, c in zip(dec["recs"], uv):
        L = line_number(r)
        if f1[0] <= L <= f1[1]:
            row = 2 * (L - f1[0])
        elif f2[0] <= L <= f2[1]:
            row = 2 * (L - f2[0]) + 1
        else:
            continue
        if row >= rows:
            continue
        Y = r["Y"]
        img[row] = yuv_rgb(Y, upsample(c["U"], px), upsample(c["V"], px)); luma[row] = Y; have[row] = True
    return img, luma, have


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["sim"])
    ap.add_argument("iq"); ap.add_argument("out")
    ap.add_argument("--src", default="parlio", choices=["parlio", "dump20", "float"])
    ap.add_argument("--cnr", type=float, default=None)
    ap.add_argument("--off", type=float, default=2.0, help="video centre - receiver LO (MHz) after removing the file's +3 MHz")
    ap.add_argument("--std", default="PAL")
    ap.add_argument("--nseg", type=int, default=32)
    ap.add_argument("--step", type=int, default=1, help="PARLIO: phase of every step-th byte")
    ap.add_argument("--amp", type=float, default=6.0)
    a = ap.parse_args()
    raw = np.fromfile(a.iq, "<i2").astype(np.float32)
    iq = (raw[0::2] + 1j * raw[1::2]) / (0.7 * 2047 * 16)
    fs = 40e6
    n = np.arange(len(iq))
    # file: video centre at +3 MHz; receiver: centre at a.off MHz from its LO; device orientation = conjugate (host negates Q)
    iq = np.conj(iq * np.exp(-2j * np.pi * (3e6 - a.off * 1e6) * n / fs))
    iq = np.concatenate([iq, iq])                           # two frames (cyclic), decode the second
    rng = np.random.default_rng(7)
    if a.src == "parlio":
        b = to_parlio(iq, a.amp, a.cnr, rng)
        phi = phase_parlio(b, a.step); rate = fs / a.step
    elif a.src == "dump20":
        w = to_dump20(iq, 100.0, a.cnr, rng)
        phi = phase_words(w); rate = 20e6
    else:
        x = iq
        if a.cnr is not None:
            p = 1 / 10 ** (a.cnr / 10)
            x = x + (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x))) * math.sqrt(p / 2)
        phi = np.angle(x); rate = fs
    std = a.std
    dec = decode(phi, rate, std, a.nseg)
    uv = chroma_uv(dec)
    T = LINE_T[std]; N = LINES[std]
    line_number = lambda r: int(round(r["t"] / T)) % N + 1
    px = 256
    img, luma, have = frame_image(dec, uv, std, px, line_number)
    save_png(a.out + "_color.png", img)
    save_png(a.out + "_luma.png", np.repeat(np.clip(luma, 0, 1)[..., None], 3, -1))
    # bar statistics: field-1 lines in the bars part (upper 40 % of the field), second frame
    f1 = (23, 310) if std == "PAL" else (22, 261)
    rows = f1[1] - f1[0] + 1
    sel = [c for r, c in zip(dec["recs"], uv) if f1[0] + 3 <= line_number(r) <= f1[0] + int(0.38 * rows)]
    st = bar_stats(sel, a.nseg)
    snr = [s["snr_db"] for s in st if s["snr_db"] is not None]
    bv = np.median([c["burst_v"] for c in uv])
    # luma noise like the VFM px 48 measurement: 48 box pixels per line, std over the bars lines (centres of the bars), in steps of 149
    selr = [r for r in dec["recs"] if f1[0] + 3 <= line_number(r) <= f1[0] + int(0.38 * rows)]
    if selr:
        Y48 = np.array([r["Y"][: (px // 48) * 48].reshape(48, -1).mean(1) if px % 48 == 0 else np.interp(np.arange(48) * px / 48, np.arange(px), r["Y"]) for r in selr])
        cols = [int((0.01 + (k + 0.5) * 0.12) * 48) for k in range(8)]
        luma_noise = float(np.median(Y48[:, cols].std(0)) * 149)
    else:
        luma_noise = None
    res = dict(src=a.src, cnr=a.cnr, std=std, lines=len(dec["recs"]), pol=dec["pol"], bars=st, chroma_snr_db_mean=round(float(np.mean(snr)), 1) if snr else None,
               burst_v_median=round(float(bv), 4), luma_noise_px48=round(luma_noise, 2) if luma_noise else None, vsw_alternation=float(np.mean(np.diff([c["vsw"] for c in uv]) != 0)))
    json.dump(res, open(a.out + "_stats.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "bars"}))
    for s in st:
        print("  ", s)


if __name__ == "__main__":
    main()
