# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Colour FM-video viewer for the ESPsoup C5 firmware (VFM with the VFMCOLOR extension).

Live:     python vfm_color_view.py live <port> [--freq MHz] [--gain 50] [--seconds 10] [--nseg 16] [--every 0]
                                         [--px auto] [--kbps 900] [--out prefix] [--show]
Offline:  python vfm_color_view.py files <luma.bin> <color.bin> <px> <nseg> <out_prefix>   (demod_host output)

Assembles the luma lines (VFL records) and the coarse colour lines (VFC records) into interlaced frames
(PAL 576 / NTSC 480 active rows), keeps the newest colour value of every row (colour arrives on every n-th line and
fills the picture over several fields), interpolates missing colour rows vertically, upsamples the nseg U/V values
per line to the luma pixels, converts Y'UV to R'G'B' and writes PNGs (luma only / colour / side by side).

Protocol (docs/espsoup-c5.md, "Colour"): VFM <rate> <D> <px> <std> <pol> <sel> <kBps> <cseg> [<every> [<step>]] adds
  four fields to the OK VFM line (nseg, every (0 = automatic), step, colour budget B/s) and frames
  VFC <seq> <t_us> <n> <2 + 2*nseg> <every> <crc32> <step>  + n x (10 + 2*nseg) bytes  (seq shared with VFL)
records [u16 line][u8 flags][u8 burst][u32 t16][u8 burst phase][u8 reference phase][nseg x (i8 U, i8 V)], U/V in luma
steps (149 = 0.7 V).  --smooth re-rotates every line with a reference tracked over many lines (PLL with drift rate on
the device's per-line reference phase) instead of the device's two-burst reference.  Measured on the C5: the per-line
reference is better (burst and chroma phase move together by ~30 degrees from line to line, probably the PARLIO
sampling phase against the modem clock), so it is off by default."""
import argparse, math, os, sys, time, zlib
import numpy as np

VF_VLOCK, VF_FIELD2, VF_VBI, VF_NTSC = 8, 2, 4, 32
VC_VNEG, VC_REF2, VC_WEAK, VC_VLOCK, VC_FIELD2, VC_NTSC = 1, 2, 4, 8, 16, 32


def parse_luma(data, n, px):
    out = []
    for j in range(n):
        o = j * (8 + px)
        out.append(dict(line=data[o] | data[o + 1] << 8, flags=data[o + 2], fc=data[o + 3], t16=int.from_bytes(data[o + 4:o + 8], "little"),
                        px=np.frombuffer(data[o + 8:o + 8 + px], np.uint8)))
    return out


def parse_color(data, n, nseg):
    out = []; rec = 10 + 2 * nseg
    for j in range(n):
        o = j * rec
        uv = np.frombuffer(data[o + 10:o + rec], np.int8).astype(np.float32)
        out.append(dict(line=data[o] | data[o + 1] << 8, flags=data[o + 2], burst=data[o + 3], t16=int.from_bytes(data[o + 4:o + 8], "little"),
                        bph=data[o + 8] * 360.0 / 256, rph=data[o + 9] * 360.0 / 256, U=uv[0::2], V=uv[1::2]))
    return out


def row_of(line, ntsc):
    """ITU line -> row of the interlaced active picture (or None)"""
    if not ntsc:
        if 23 <= line <= 310: return 2 * (line - 23)
        if 336 <= line <= 623: return 2 * (line - 336) + 1
        return None
    if 22 <= line <= 261: return 2 * (line - 22)
    if 285 <= line <= 524: return 2 * (line - 285) + 1
    return None


class Picture:
    def __init__(self, px, nseg, ntsc=False):
        self.ntsc = ntsc; self.rows = 480 if ntsc else 576; self.px = px; self.nseg = nseg
        self.Y = np.zeros((self.rows, px), np.float32); self.yseen = np.zeros(self.rows, bool)
        self.U = np.zeros((self.rows, nseg), np.float32); self.V = np.zeros((self.rows, nseg), np.float32)
        self.cage = np.full(self.rows, -1, np.int64); self.burst = []; self.cnt = 0
        self.acc = False
        self.vsmooth = 6; self.smooth = False; self.th = None; self.w = 0.0; self.t_last = 0; self.vfix = 0

    def add_luma(self, L):
        if not L["flags"] & VF_VLOCK: return
        r = row_of(L["line"], self.ntsc)
        if r is None or r >= self.rows: return
        self.Y[r] = np.clip((L["px"].astype(np.float32) - 64) / 149, -0.1, 1.2); self.yseen[r] = True

    def resmooth(self, C):
        """Reference PLL (phase + drift rate) over the device's per-line reference phases rph (deg; already free of the
        PAL +-45 degree swing: the sum of two neighbouring bursts).  The subcarrier of the transmitter drifts against
        the receiver's oscillator (crystals: a few deg per line), so the PLL tracks the rate too.  The line is
        re-rotated from rph to the tracked reference, the PAL V switch is re-decided from the burst phase bph."""
        pal = not C["flags"] & VC_NTSC
        meas = C["rph"]
        if self.th is None or ((C["t16"] - self.t_last) % 2 ** 32) / 1024.0 > 20000:
            self.th = meas; self.w = 0.0; dt = 0.0
        else:
            dt = ((C["t16"] - self.t_last) % 2 ** 32) / 1024.0      # lines since the last colour record
            pred = self.th + self.w * dt
            e = (meas - pred + 180) % 360 - 180
            self.th = (pred + 0.25 * e) % 360; self.w += 0.03 * e / max(1.0, dt)
        self.t_last = C["t16"]
        ref = self.th
        d = np.radians((ref - C["rph"] + 180) % 360 - 180)
        vsw_dev = -1 if C["flags"] & VC_VNEG else 1
        re, im = vsw_dev * C["V"], -C["U"]                   # S' of the device
        re2, im2 = re * np.cos(d) + im * np.sin(d), im * np.cos(d) - re * np.sin(d)   # S' * exp(-j d)
        vsw = (1 if ((C["bph"] - ref + 180) % 360 - 180) < 0 else -1) if pal else 1
        if vsw != vsw_dev: self.vfix += 1
        return -im2, vsw * re2

    def add_color(self, C):
        if not C["flags"] & VC_VLOCK or C["flags"] & VC_WEAK: return
        if self.smooth:
            C = dict(C); C["U"], C["V"] = self.resmooth(C)
        r = row_of(C["line"], self.ntsc)
        if r is None or r >= self.rows: return
        self.U[r] = C["U"] / 149; self.V[r] = C["V"] / 149; self.cnt += 1; self.cage[r] = self.cnt
        self.burst.append(C["burst"])

    def rgb(self, sat=1.0, acc=False, nominal_burst=32.0):
        have = self.cage >= 0
        U, V = self.U.copy(), self.V.copy()
        if have.sum() >= 2:
            idx = np.nonzero(have)[0]
            for k in range(self.nseg):                       # vertical interpolation of rows without colour yet
                U[:, k] = np.interp(np.arange(self.rows), idx, self.U[idx, k]); V[:, k] = np.interp(np.arange(self.rows), idx, self.V[idx, k])
            # vertical chroma low-pass over vsmooth rows (PAL delay-line effect: hue errors of neighbouring lines cancel; the
            # per-line hue noise of the coarse colour, ~30 deg rms on the C5 PARLIO data, averages down)
            k = max(1, int(self.vsmooth)); ker = np.ones(k) / k
            U = np.apply_along_axis(lambda c: np.convolve(np.pad(c, (k // 2, k - 1 - k // 2), mode="edge"), ker, "valid"), 0, U)
            V = np.apply_along_axis(lambda c: np.convolve(np.pad(c, (k // 2, k - 1 - k // 2), mode="edge"), ker, "valid"), 0, V)
        g = sat
        if acc and self.burst:
            g *= nominal_burst / max(1.0, float(np.median(self.burst[-500:])))
        xc = (np.arange(self.nseg) + 0.5) / self.nseg; xp = (np.arange(self.px) + 0.5) / self.px
        Uu = np.array([np.interp(xp, xc, u) for u in U]) * g; Vu = np.array([np.interp(xp, xc, v) for v in V]) * g
        Y = self.Y
        B = Y + Uu / 0.493; R = Y + Vu / 0.877; G = (Y - 0.299 * R - 0.114 * B) / 0.587
        return np.clip(np.stack([R, G, B], -1), 0, 1)

    def save(self, prefix, scale_w=None, acc=False):
        from PIL import Image
        w = scale_w or max(self.px, 512)
        lum = Image.fromarray((np.clip(self.Y, 0, 1) * 255).astype(np.uint8)).resize((w, self.rows), Image.BILINEAR).convert("RGB")
        col = Image.fromarray((self.rgb(acc=acc) * 255).astype(np.uint8)).resize((w, self.rows), Image.BILINEAR)
        lum.save(prefix + "_luma.png"); col.save(prefix + "_color.png")
        both = Image.new("RGB", (2 * w + 8, self.rows), (40, 40, 40)); both.paste(lum, (0, 0)); both.paste(col, (w + 8, 0))
        both.save(prefix + "_compare.png")
        return prefix + "_color.png"


# ------------------------------------------------------------------ live
def auto_px(kbps, every, nseg, lines_per_s=14200):
    """luma pixels that leave room for the colour records in the USB budget (all active lines)"""
    col = lines_per_s / max(1, every) * (10 + 2 * nseg) if every else kbps * 1000 / 8   # the device gives colour 1/8 of kBps
    px = int((kbps * 1000 - col) / lines_per_s) - 8
    return max(16, min(128, px))


def live(a):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from vfm_color_capture import Dev
    d = Dev(a.port)
    caps = d.ask("CAPS").split()
    if "VFMCOLOR" not in caps:
        raise SystemExit("firmware without VFMCOLOR: " + d.ask("FWOPT?"))
    if a.freq:
        print(d.ask(f"FREQ {int(a.freq)}"))
    print(d.ask(f"GAIN MANUAL {a.gain}"))
    px = auto_px(a.kbps, a.every, a.nseg) if a.px == "auto" else int(a.px)
    cmd = f"VFM 2 0 {px} {a.std} {a.pol} 1 {a.kbps} {a.nseg} {a.every} {a.step}"
    print(">", cmd)
    d.cmd(cmd)
    ok = d.line(10)
    print(ok)
    if not ok.startswith("OK VFM"):
        raise SystemExit(ok)
    pic = None; t_end = time.time() + a.seconds; nl = nc = 0; nb = 0; everys = []; t0 = time.time(); shots = 0
    last_show = 0; plt = None
    if a.show:
        import matplotlib.pyplot as plt
        plt.ion(); fig, ax = plt.subplots(); im = None
    stop_sent = False; rawc = []; rawl = []
    while True:
        if time.time() > t_end and not stop_sent:
            d.cmd("STOP"); stop_sent = True
        h = d.line(10).split()
        if h[0] == "END":
            print(" ".join(h)); break
        if h[0] == "VFL":
            n, p = int(h[3]), int(h[4]); data = d.read(n * (8 + p)); nb += len(data) + 60
            if zlib.crc32(data) != int(h[6], 16): print("CRC VFL"); continue
            st = int(h[9])
            if pic is None and st & 1:
                pic = Picture(p, a.nseg, ntsc=bool(st & 4)); pic.smooth = a.smooth; pic.vsmooth = a.vsmooth
            for L in parse_luma(data, n, p):
                if pic: pic.add_luma(L)
                if a.save_raw: rawl.append(L)
            nl += n
        elif h[0] == "VFC":
            n, ns = int(h[3]), (int(h[4]) - 2) // 2; data = d.read(n * (10 + 2 * ns)); nb += len(data) + 50
            if zlib.crc32(data) != int(h[6], 16): print("CRC VFC"); continue
            everys.append(int(h[5]))
            for C in parse_color(data, n, ns):
                if pic: pic.add_color(C)
                if a.save_raw: rawc.append(C)
            nc += n
        else:
            print("?", " ".join(h)[:80]); continue
        if a.show and pic is not None and time.time() - last_show > 0.5:
            last_show = time.time(); rgb = pic.rgb(acc=a.acc)
            if im is None: im = ax.imshow(rgb, aspect="auto")
            else: im.set_data(rgb)
            plt.pause(0.001)
        if a.snap and pic is not None and time.time() - t0 > (shots + 1) * a.snap:
            shots += 1; pic.save(f"{a.out}_t{shots:02d}", acc=a.acc)
    dt = time.time() - t0
    dstat = d.ask("DSTAT?"); pstat = d.ask("PSTAT?")
    d.close()
    res = dict(cmd=cmd, px=px, seconds=round(dt, 1), luma_lines_per_s=round(nl / dt), color_lines_per_s=round(nc / dt), kB_per_s=round(nb / dt / 1000, 1),
               every_median=float(np.median(everys)) if everys else None, every_minmax=[min(everys), max(everys)] if everys else None,
               dstat=dstat, pstat=pstat)
    print(res)
    if pic is not None:
        res["png"] = pic.save(a.out, acc=a.acc)
        res["burst_median"] = float(np.median(pic.burst)) if pic.burst else None
        res["rows_luma"] = int(pic.yseen.sum()); res["rows_color"] = int((pic.cage >= 0).sum())
    import json
    json.dump(res, open(a.out + "_live.json", "w"), indent=1)
    if a.save_raw:
        cv = lambda d: {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in d.items()}
        json.dump(dict(luma=[cv(x) for x in rawl], color=[cv(x) for x in rawc]), open(a.out + "_raw.json", "w"))
    return res


def files(a):
    px, nseg = int(a.px), int(a.nseg)
    lb = open(a.luma, "rb").read(); cb = open(a.color, "rb").read()
    rl, rc = 4 + 8 + px, 4 + 10 + 2 * nseg
    L = parse_luma(b"".join(lb[k + 4:k + rl] for k in range(0, len(lb), rl)), len(lb) // rl, px)
    C = parse_color(b"".join(cb[k + 4:k + rc] for k in range(0, len(cb), rc)), len(cb) // rc, nseg)
    ntsc = bool(L and L[0]["flags"] & VF_NTSC)
    pic = Picture(px, nseg, ntsc); pic.smooth = a.smooth; pic.vsmooth = a.vsmooth
    for x in L: pic.add_luma(x)
    for x in C: pic.add_color(x)
    print("luma lines", len(L), "colour lines", len(C), "rows", int(pic.yseen.sum()), int((pic.cage >= 0).sum()),
          "burst median", float(np.median(pic.burst)) if pic.burst else None)
    print(pic.save(a.out, acc=a.acc))
    return pic, L, C


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="mode", required=True)
    l = sp.add_parser("live"); l.add_argument("port"); l.add_argument("--freq", type=float); l.add_argument("--gain", type=int, default=50)
    l.add_argument("--seconds", type=float, default=10); l.add_argument("--nseg", type=int, default=16); l.add_argument("--every", type=int, default=0)
    l.add_argument("--step", type=int, default=1); l.add_argument("--px", default="auto"); l.add_argument("--kbps", type=int, default=900)
    l.add_argument("--std", type=int, default=0); l.add_argument("--pol", type=int, default=0)
    l.add_argument("--out", default="vfm_color"); l.add_argument("--show", action="store_true"); l.add_argument("--snap", type=float, default=0)
    l.add_argument("--acc", action="store_true", help="automatic colour control: scale chroma by nominal / measured burst")
    l.add_argument("--smooth", action="store_true", help="re-rotate with a reference PLL over many lines (helps on clean simulated signals; on "
                   "the C5 PARLIO data the per-line reference is better: burst and chroma phase move together from line to line)")
    l.add_argument("--vsmooth", type=int, default=6, help="vertical chroma low-pass (rows of the frame, default 6)")
    l.add_argument("--save-raw", dest="save_raw", action="store_true", help="also store all records (<out>_raw.json)")
    f = sp.add_parser("files"); f.add_argument("luma"); f.add_argument("color"); f.add_argument("px"); f.add_argument("nseg"); f.add_argument("out")
    f.add_argument("--acc", action="store_true"); f.add_argument("--smooth", action="store_true"); f.add_argument("--vsmooth", type=int, default=6)
    a = ap.parse_args()
    live(a) if a.mode == "live" else files(a)


if __name__ == "__main__":
    main()
