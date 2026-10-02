# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Offline colour analysis of raw captures (vfm_color_capture.py) of the 75 % colour-bar test picture.

  python vfm_color_analyse.py <capture.npz> <out_prefix> [--nseg 32] [--step 1]

Decodes every PARLIO (PCAP) and 10-bit (CAPT20) snapshot with vfm_color_ref.py, finds the colour-bar lines by their
luma staircase, and reports per bar the mean U/V, the chroma noise and the chroma SNR (75 % bar amplitude / rms
noise per segment), hue error and saturation; writes a strip image of all decoded lines (luma | colour)."""
import argparse, json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vfm_color_ref as R

BAR_Y = np.array([0.75, 0.665, 0.525, 0.440, 0.310, 0.224, 0.0855, 0.0])


def is_bars(Y):
    px = len(Y)
    yc = np.array([np.median(Y[int((0.01 + (k + 0.3) * 0.12) * px):int((0.01 + (k + 0.7) * 0.12) * px)]) for k in range(8)])
    return np.max(np.abs(yc - BAR_Y)) < 0.12, yc


def analyse(snaps, fs, phase_fn, nseg, label, std="PAL"):
    lines, bars, ys = [], [], []
    for s in snaps:
        dec = R.decode(phase_fn(s), fs, std, nseg)
        uv = R.chroma_uv(dec)
        for r, c in zip(dec["recs"], uv):
            ok, yc = is_bars(r["Y"])
            lines.append((r["Y"], c["U"], c["V"]))
            if ok:
                bars.append(c); ys.append(yc)
    st = R.bar_stats(bars, nseg) if bars else []
    snr = [x["snr_db"] for x in st if x["snr_db"] is not None]
    bv = float(np.median([c["burst_v"] for c in bars])) if bars else None
    # luma noise in the bars: std over lines of the bar-centre medians, in steps of 149
    ln = float(np.median(np.std(np.array(ys), 0)) * 149) if len(ys) > 2 else None
    res = dict(source=label, lines=len(lines), bar_lines=len(bars), chroma_snr_db=round(float(np.mean(snr)), 1) if snr else None,
               chroma_noise_steps=round(float(np.mean([x["noise"] for x in st])) * 149, 2) if st else None,
               burst_v=round(bv, 4) if bv else None, luma_noise_steps=round(ln, 2) if ln else None,
               vsw_alt=round(float(np.mean(np.diff([c["vsw"] for c in bars]) != 0)), 3) if len(bars) > 2 else None, bars=st)
    return res, lines


def strip(lines, px=256):
    rows = []
    for Y, U, V in lines:
        Yp = np.interp(np.linspace(0, len(Y) - 1, px), np.arange(len(Y)), Y)
        rows.append(R.yuv_rgb(Yp, R.upsample(U, px), R.upsample(V, px)))
    rgb = np.array(rows)
    lum = np.repeat(np.clip(np.array([np.interp(np.linspace(0, len(Y) - 1, px), np.arange(len(Y)), Y) for Y, _, _ in lines]), 0, 1)[..., None], 3, -1)
    return np.concatenate([lum, np.full((len(lines), 6, 3), 0.15), rgb], 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--nseg", type=int, default=32); ap.add_argument("--step", type=int, default=1); ap.add_argument("--std", default="PAL")
    a = ap.parse_args()
    z = np.load(a.inp)
    out = {}
    if len(z["pcap"]):
        res, lines = analyse(z["pcap"], 40e6 / a.step, lambda b: R.phase_parlio(b, a.step), a.nseg, f"PARLIO 4+4 bit 40 MS/s (lsb {int(z['lsb'])}, step {a.step})", a.std)
        out["parlio"] = res
        R.save_png(a.out + "_parlio_strip.png", strip(lines[:200]), (2, 2))
    if len(z["capt"]):
        res, lines = analyse(z["capt"], 20e6, R.phase_words, a.nseg, "dump 10 bit 20 MS/s", a.std)
        out["dump20"] = res
        R.save_png(a.out + "_dump20_strip.png", strip(lines[:200]), (2, 2))
    json.dump(out, open(a.out + "_analysis.json", "w"), indent=1, default=float)
    for k, v in out.items():
        print(k, {x: y for x, y in v.items() if x != "bars"})
        for b in v["bars"]:
            print("   ", b["bar"], "U", b["U"], "V", b["V"], "noise", b["noise"], "snr", b["snr_db"], "hue", b["hue_err_deg"], "sat", b["sat_ratio"])


if __name__ == "__main__":
    main()
