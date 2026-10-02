# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Hardware checks of VFM colour (VFMCOLOR) on an ESPsoup C5 board with an FM colour test picture on the air.

  python vfm_color_hw.py <port> <out_dir> --freq 5806 --gain 66 [--plsb 3] [--host demod_host.exe] [--seconds 10]

1. VCDATA: one PARLIO snapshot + the device's VFM/colour records of it; the host build (tests/espsoup/demod_host.c,
   same DSP source) recomputes them from the same bytes -> must be byte-identical (t16 shifted by the snapshot time).
2. Live runs (vfm_color_view.py): luma only (reference) and several colour configurations; lines/s, colour lines/s,
   kB/s, CPU cycles (DSTAT?), PARLIO duty (PSTAT?), PNGs.  Results in <out_dir>/hw_result.json."""
import argparse, json, os, subprocess, sys, tempfile, time, zlib
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from vfm_color_capture import Dev
import vfm_color_view as V


def vcdata(d, host, args, tmp):
    d.cmd("VCDATA " + args)
    h = d.line(10).split()
    if h[0] != "PDATA":
        raise RuntimeError(" ".join(h))
    n, t0 = int(h[1]), int(h[3]); raw = d.read(n)
    if zlib.crc32(raw) != int(h[2], 16):
        raise RuntimeError("PDATA CRC")
    x = d.line(10).split()
    if x[0] != "VCX":
        raise RuntimeError(" ".join(x))
    nl, px, nc, nseg = int(x[1]), int(x[2]), int(x[4]), int(x[5])
    lum = d.read(nl * (8 + px)); col = d.read(nc * (10 + 2 * nseg))
    if zlib.crc32(lum) != int(x[3], 16) or zlib.crc32(col) != int(x[6], 16):
        raise RuntimeError("VCX CRC")
    every, step = int(x[7]), int(x[8])
    f = os.path.join(tmp, "vc.u8"); open(f, "wb").write(raw)
    a = args.split()
    D, px_, st, pol, sel = a[0], a[1], a[2], a[3], a[4]
    hpol = {"0": "0", "1": "2", "2": "1"}[pol]                 # host harness takes the device-domain pol_force
    lo, co = os.path.join(tmp, "l.bin"), os.path.join(tmp, "c.bin")
    out = subprocess.run([host, "vfmp", f, lo, "0", "57344", "0", D, px_, st, hpol, sel, "0", str(nseg), str(every), str(step), co], capture_output=True, text=True)
    hl, hc = open(lo, "rb").read(), open(co, "rb").read()
    strip = lambda b, rec: b"".join(b[k + 4:k + rec] for k in range(0, len(b), rec))
    hl, hc = strip(hl, 4 + 8 + px), strip(hc, 4 + 10 + 2 * nseg)
    off = (t0 * 16) & 0xffffffff
    def shift(b, rec):                                          # device t16 = host t16 + t0 * 16
        b = bytearray(b)
        for k in range(0, len(b), rec):
            t = (int.from_bytes(b[k + 4:k + 8], "little") - off) & 0xffffffff; b[k + 4:k + 8] = t.to_bytes(4, "little")
        return bytes(b)
    dl, dc = shift(lum, 8 + px), shift(col, 10 + 2 * nseg)
    return dict(args=args, luma_lines=nl, color_lines=nc, luma_equal=dl == hl, color_equal=dc == hc, host_luma=len(hl) // (8 + px),
                host_color=len(hc) // (10 + 2 * nseg), host=out.stdout.strip().splitlines()[-1] if out.stdout else out.stderr)


class Args:
    pass


def live(port, out, tag, freq, gain, seconds, nseg, every, step, px, kbps):
    a = Args(); a.port, a.freq, a.gain, a.seconds, a.nseg, a.every, a.step, a.px, a.kbps = port, None, gain, seconds, nseg, every, step, str(px), kbps
    a.std, a.pol, a.out, a.show, a.snap, a.acc, a.smooth, a.save_raw, a.vsmooth = 1, 1, os.path.join(out, tag), False, 0, False, False, True, 6
    return V.live(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port"); ap.add_argument("out"); ap.add_argument("--freq", type=float, default=5806); ap.add_argument("--gain", type=int, default=66)
    ap.add_argument("--plsb", default="3"); ap.add_argument("--host", default=os.environ.get("DEMOD_HOST", "demod_host"))
    ap.add_argument("--seconds", type=float, default=10); ap.add_argument("--skip-live", action="store_true"); ap.add_argument("--only", default=""); ap.add_argument("--vcrep", type=int, default=3)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    res = {}
    d = Dev(a.port)
    res["fw"] = d.ask("FWOPT?"); res["caps_has_vfmcolor"] = "VFMCOLOR" in d.ask("CAPS").split()
    print(res)
    print(d.ask(f"FREQ {int(a.freq)}"), d.ask(f"GAIN MANUAL {a.gain}"), d.ask(f"PLSB {a.plsb}"))
    with tempfile.TemporaryDirectory() as tmp:
        res["vcdata"] = []
        for args in ["0 64 0 0 1 0 16 1 1", "0 48 0 0 1 0 32 4 2", "0 48 0 0 1 0 16 2 4", "0 40 0 0 1 0 8 1 2"] * a.vcrep:
            r = vcdata(d, a.host, args, tmp); res["vcdata"].append(r); print("VCDATA", {k: r[k] for k in ("args", "luma_lines", "color_lines", "luma_equal", "color_equal")})
    print(d.ask("PLSB AUTO"))
    d.close()
    if not a.skip_live:
        runs = [("luma40", 0, 0, 2, 40, 900), ("luma32", 0, 0, 2, 32, 900), ("auto_n16_s2_px40", 16, 0, 2, 40, 900), ("auto_n16_s4_px32", 16, 0, 4, 32, 900),
                ("e16_n16_s4_px32", 16, 16, 4, 32, 900), ("e16_n32_s2_px32", 32, 16, 2, 32, 900), ("e8_n16_s4_px24", 16, 8, 4, 24, 900),
                ("auto_n16_s2_px128", 16, 0, 2, 128, 900)]
        for tag, nseg, every, step, px, kb in runs:
            if a.only and tag not in a.only.split(","):
                continue
            if nseg == 0:
                r = luma_only(a.port, a.out, tag, a.gain, a.seconds, px, kb)
            else:
                r = live(a.port, a.out, tag, a.freq, a.gain, a.seconds, nseg, every, step, px, kb)
            res[tag] = r
            json.dump(res, open(os.path.join(a.out, "hw_result.json"), "w"), indent=1, default=str)
    json.dump(res, open(os.path.join(a.out, "hw_result.json"), "w"), indent=1, default=str)


def luma_only(port, out, tag, gain, seconds, px, kbps, pol=0):
    """reference run without colour (exactly the old VFM command)"""
    d = Dev(port)
    print(d.ask(f"GAIN MANUAL {gain}"))
    d.cmd(f"VFM 2 0 {px} 0 {pol} 1 {kbps}")
    ok = d.line(10); print(ok)
    t0 = time.time(); nl = nb = 0; stop = False; pic = None; frames = 0
    while True:
        if not stop and time.time() - t0 > seconds:
            d.cmd("STOP"); stop = True
        h = d.line(10).split()
        if h[0] == "END":
            end = " ".join(h); break
        if h[0] != "VFL":
            continue
        n, p = int(h[3]), int(h[4]); data = d.read(n * (8 + p)); nb += len(data) + 60; nl += n
        if pic is None and int(h[9]) & 1:
            pic = V.Picture(p, 16, bool(int(h[9]) & 4))
        if pic:
            for L in V.parse_luma(data, n, p):
                pic.add_luma(L)
    dt = time.time() - t0
    r = dict(cmd=f"VFM 2 0 {px} 0 {pol} 1 {kbps}", end=end, seconds=round(dt, 1), luma_lines_per_s=round(nl / dt), kB_per_s=round(nb / dt / 1000, 1),
             fps_equiv=round(nl / dt / 576, 1), dstat=d.ask("DSTAT?"), pstat=d.ask("PSTAT?"))
    d.close()
    if pic:
        r["png"] = pic.save(os.path.join(out, tag))
    print(tag, r)
    return r


if __name__ == "__main__":
    main()
