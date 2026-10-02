# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Colour from raw snapshots without any firmware change (prototype of the app-side alternative).

  python vfm_color_snapshots.py capture <port> <out.npz> [--freq MHz] [--gain 50] [--n 40] [--kind pcap|capt] [--plsb 2]
  python vfm_color_snapshots.py decode <in.npz> <out_prefix> [--nseg 32]

capture: a short VFM run (luma line stream: ITU line number + sync time t16 of every line), then n raw snapshots
(PCAP 1.64 ms at 40 MS/s 4+4 bit, or CAPT20 1.64 ms at 20 MS/s 10 bit) with their start time t0 (us since boot,
same clock), then a second VFM run.  decode: line timing model (line period and phase from the two VFM runs) ->
every snapshot line gets its ITU line number from its time alone -> the colour of the ~25 lines per snapshot is
decoded (vfm_color_ref.py) and painted into a colour layer over the newest luma picture, which fills up snapshot by
snapshot (the app would do the same with BIGCAP snapshots between VFM bursts)."""
import argparse, os, sys, time, zlib
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vfm_color_ref as R
from vfm_color_capture import Dev


def vfm_lines(d, seconds, px=48):
    d.cmd(f"VFM 2 0 {px} 0 0 1 800")
    ok = d.line(10)
    if not ok.startswith("OK VFM"):
        raise RuntimeError(ok)
    out = []; t_end = time.time() + seconds; stop = False
    while True:
        if not stop and time.time() > t_end:
            d.cmd("STOP"); stop = True
        h = d.line(10).split()
        if h[0] == "END":
            break
        if h[0] != "VFL":
            continue
        n, p = int(h[3]), int(h[4]); data = d.read(n * (8 + p))
        if zlib.crc32(data) != int(h[6], 16):
            continue
        for j in range(n):
            o = j * (8 + p)
            out.append((data[o] | data[o + 1] << 8, data[o + 2], data[o + 3], int.from_bytes(data[o + 4:o + 8], "little"),
                        np.frombuffer(data[o + 8:o + 8 + p], np.uint8).copy()))
    return out


def capture(a):
    d = Dev(a.port)
    if a.freq:
        print(d.ask(f"FREQ {int(a.freq)}") if float(a.freq).is_integer() else d.ask(f"FREQK {int(round(a.freq * 1000))}"))
    print(d.ask(f"GAIN MANUAL {a.gain}"))
    L1 = vfm_lines(d, 1.5)
    print(d.ask(f"PLSB {a.plsb}"))
    snaps, t0s = [], []
    for k in range(a.n):
        if a.kind == "pcap":
            b, t0, _ = d.pcap(); snaps.append(b)
        else:
            w, t0 = d.capt20(); snaps.append(w)
        t0s.append(t0)
    print(d.ask("PLSB AUTO"))
    L2 = vfm_lines(d, 1.5)
    d.close()
    pack = lambda L: (np.array([x[0] for x in L]), np.array([x[1] for x in L]), np.array([x[2] for x in L]), np.array([x[3] for x in L], np.uint32),
                      np.array([x[4] for x in L]))
    a1, a2 = pack(L1), pack(L2)
    np.savez_compressed(a.out, kind=a.kind, snaps=np.array(snaps), t0=np.array(t0s, np.int64), plsb=a.plsb,
                        l1_line=a1[0], l1_flags=a1[1], l1_fc=a1[2], l1_t16=a1[3], l1_px=a1[4], l2_line=a2[0], l2_flags=a2[1], l2_fc=a2[2], l2_t16=a2[3], l2_px=a2[4])
    print("saved", a.out, len(L1), len(L2), "lines", len(snaps), a.kind)


def timing_model(t16, line, flags, nlines=625):
    """Line clock of one VFM run, robust against wrong line numbers and relocks: the line period T (1/16 us) from a
    least-squares fit of the sync times over integer line counts (outliers removed), the frame phase from a vote of
    (t16 - (line - 1) * T) mod frame over the VLOCK lines.  Returns (phase, T, rms residual): line(t) =
    round((t - phase) / T) mod nlines + 1, t in 1/16 us (mod 2^32 handled by the caller)."""
    t = np.unwrap(t16.astype(np.float64), period=2 ** 32)
    k = np.round((t - t[0]) / 1024.0)
    for _ in range(3):
        A = np.vstack([np.ones_like(k), k]).T
        c, *_ = np.linalg.lstsq(A, t, rcond=None)
        res = t - A @ c
        good = np.abs(res) < max(8.0, 4 * np.median(np.abs(res)))
        k, t = k[good], t[good]; flags, line = flags[good], line[good]
        k = np.round((t - c[0]) / c[1])
    T = c[1]
    ok = (flags & 8) > 0
    frame = nlines * T
    ph = np.mod(t[ok] - (line[ok].astype(np.float64) - 1) * T, frame)
    hist, edges = np.histogram(ph, bins=int(nlines), range=(0, frame))
    j = int(np.argmax(hist))
    sel = np.abs(np.mod(ph - (edges[j] + T / 2) + frame / 2, frame) - frame / 2) < T
    phase = (edges[j] + T / 2 + np.mean(np.mod(ph[sel] - (edges[j] + T / 2) + frame / 2, frame) - frame / 2)) % frame
    return phase, T, float(np.std(res[good])), float(hist[j] / max(1, ok.sum()))


def line_at(t16, phase, T, nlines=625):
    frame = nlines * T
    return int(np.round(np.mod(t16 - phase, frame) / T)) % nlines + 1


def decode(a):
    z = np.load(a.inp)
    kind = str(z["kind"]); nl = 625
    ph_a, T_a, r_a, v_a = timing_model(z["l1_t16"], z["l1_line"], z["l1_flags"])
    ph_b, T_b, r_b, v_b = timing_model(z["l2_t16"], z["l2_line"], z["l2_flags"])
    print(f"VFM run 1: T = {T_a / 16:.5f} us (resid {r_a / 16:.3f} us, vote {100 * v_a:.0f} %), run 2: T = {T_b / 16:.5f} us (resid {r_b / 16:.3f} us, vote {100 * v_b:.0f} %)")
    # frame phase is defined modulo 2^32 of t16: compare the prediction of run 1 with run 2 at the first line of run 2
    T = T_a
    t2 = float(z["l2_t16"][0])
    # unwrap: phases are in the t16 domain of each run (wraps every 268 s; runs are seconds apart -> same wrap)
    pred = line_at(t2, ph_a, T); meas = line_at(t2, ph_b, T_b)
    print("prediction check at the start of run 2: run-1 model line", pred, "run-2 model line", meas)
    # luma picture from run 2 (newest)
    px = z["l2_px"].shape[1]
    Y = np.zeros((576, px)); seen = np.zeros(576, bool)
    for ln, fl, p in zip(z["l2_line"], z["l2_flags"], z["l2_px"]):
        r = row_of(ln)
        if fl & 8 and r is not None:
            Y[r] = np.clip((p.astype(float) - 64) / 149, 0, 1.2); seen[r] = True
    U = np.zeros((576, a.nseg)); V = np.zeros((576, a.nseg)); cseen = np.zeros(576, bool)
    fill = []
    for k, (snap, t0) in enumerate(zip(z["snaps"], z["t0"])):
        if kind == "pcap":
            phi = R.phase_parlio(snap); fs = 40e6
        else:
            phi = R.phase_words(snap); fs = 20e6
        dec = R.decode(phi, fs, "PAL", a.nseg)
        uv = R.chroma_uv(dec)
        for r, c in zip(dec["recs"], uv):
            t16 = (int(t0) * 16 + int(round(r["edge"] / fs * 16e6))) % 2 ** 32
            ln = line_at(t16, ph_a, T)
            row = row_of(ln)
            if row is None or c["burst_v"] < 0.01:
                continue
            U[row] = c["U"]; V[row] = c["V"]; cseen[row] = True
        fill.append(float(cseen.mean()))
    print("colour rows after each snapshot:", " ".join(f"{100 * f:.0f}%" for f in fill[:: max(1, len(fill) // 10)]), f"-> {100 * fill[-1]:.1f} %")
    from PIL import Image
    idx = np.nonzero(cseen)[0]
    Ui, Vi = U.copy(), V.copy()
    if len(idx) >= 2:
        for j in range(a.nseg):
            Ui[:, j] = np.interp(np.arange(576), idx, U[idx, j]); Vi[:, j] = np.interp(np.arange(576), idx, V[idx, j])
    up = lambda M: np.array([R.upsample(m, px) for m in M])
    rgb = R.yuv_rgb(Y, up(Ui), up(Vi))
    w = 512
    lum = Image.fromarray((np.clip(Y, 0, 1) * 255).astype(np.uint8)).resize((w, 576), Image.BILINEAR).convert("RGB")
    col = Image.fromarray((rgb * 255).astype(np.uint8)).resize((w, 576), Image.BILINEAR)
    both = Image.new("RGB", (2 * w + 8, 576), (40, 40, 40)); both.paste(lum, (0, 0)); both.paste(col, (w + 8, 0))
    both.save(a.out + "_compare.png"); col.save(a.out + "_color.png")
    print("saved", a.out + "_compare.png")


def row_of(line):
    if 23 <= line <= 310: return 2 * (line - 23)
    if 336 <= line <= 623: return 2 * (line - 336) + 1
    return None


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="mode", required=True)
    c = sp.add_parser("capture"); c.add_argument("port"); c.add_argument("out"); c.add_argument("--freq", type=float); c.add_argument("--gain", type=int, default=50)
    c.add_argument("--n", type=int, default=40); c.add_argument("--kind", default="pcap"); c.add_argument("--plsb", default="2")
    d = sp.add_parser("decode"); d.add_argument("inp"); d.add_argument("out"); d.add_argument("--nseg", type=int, default=32)
    a = ap.parse_args()
    capture(a) if a.mode == "capture" else decode(a)


if __name__ == "__main__":
    main()
