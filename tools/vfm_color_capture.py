# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Raw captures of an FM video signal for the offline colour analysis (vfm_color_ref.py).

  python vfm_color_capture.py <port> <out.npz> <freq_mhz> [--gain 50] [--pcap 8] [--capt 8] [--plsb auto|0..6]

Takes PARLIO snapshots (PCAP 65532 bytes = 1.64 ms at 40 MS/s, 4+4 bit) and 10-bit dump snapshots (CAPT20 32704
samples = 1.64 ms at 20 MS/s), each with its t0 (us since boot), and stores them with the settings.
Needs an ESPsoup C5 firmware with the CAPS tokens PARLIO and CAPT."""
import argparse, sys, time, zlib
import numpy as np
import serial


class Dev:
    def __init__(self, port):
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, 2000000, 0.05
        self.s.dtr = False; self.s.rts = False
        self.s.open(); self.buf = bytearray(); self.sync()

    def _fill(self, dl):
        c = self.s.read(self.s.in_waiting or 1)
        if c:
            self.buf += c
        elif time.time() > dl:
            raise TimeoutError("serial timeout")

    def read(self, n, timeout=10):
        dl = time.time() + timeout
        while len(self.buf) < n:
            self._fill(dl)
        out, self.buf = bytes(self.buf[:n]), self.buf[n:]
        return out

    def line(self, timeout=5):
        dl = time.time() + timeout
        while b"\n" not in self.buf:
            self._fill(dl)
        i = self.buf.index(b"\n"); out, self.buf = bytes(self.buf[:i]), self.buf[i + 1:]
        return out.decode(errors="replace").strip()

    def cmd(self, s):
        self.s.write((s + "\n").encode())

    def ask(self, s, timeout=5):
        self.cmd(s); return self.line(timeout)

    def sync(self):
        for _ in range(4):
            m = f"SYNC {int(time.time() * 1000)}"
            self.s.reset_input_buffer(); self.buf.clear(); self.cmd("\n" + m)
            dl = time.time() + 2
            try:
                while True:
                    self._fill(dl)
                    k = (m + "\n").encode()
                    if k in self.buf:
                        self.buf = self.buf[self.buf.index(k) + len(k):]; return
            except TimeoutError:
                pass
        raise RuntimeError("sync failed")

    def pcap(self, n=65532):
        self.cmd(f"PCAP {n}")
        h = self.line(5).split()
        if h[0] != "PDATA":
            raise RuntimeError(" ".join(h))
        d = self.read(int(h[1]))
        if zlib.crc32(d) != int(h[2], 16):
            raise RuntimeError("CRC")
        return np.frombuffer(d, np.uint8).copy(), int(h[3]), int(h[4])

    def capt20(self, n=32704, rate_idx=2):
        self.cmd(f"CAPT20 {n} {rate_idx}")
        h = self.line(5).split()
        if h[0] != "DATAT":
            raise RuntimeError(" ".join(h))
        n = int(h[1]); d = self.read((n * 20 + 7) // 8)
        if zlib.crc32(d) != int(h[2], 16):
            raise RuntimeError("CRC")
        bits = np.unpackbits(np.frombuffer(d, np.uint8), bitorder="little")[: n * 20].reshape(n, 20)
        w = (bits.astype(np.uint32) << np.arange(20, dtype=np.uint32)).sum(1).astype(np.uint32)
        return w, int(h[4])

    def close(self):
        try:
            self.ask("RELEASE", 1)
        except Exception:
            pass
        self.s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port"); ap.add_argument("out"); ap.add_argument("freq", type=float)
    ap.add_argument("--gain", type=int, default=50)
    ap.add_argument("--pcap", type=int, default=8); ap.add_argument("--capt", type=int, default=8)
    ap.add_argument("--plsb", default="auto")
    ap.add_argument("--dccal", action="store_true", help="hardware DC calibration after GAIN (lasts until FREQ/GAIN)")
    a = ap.parse_args()
    d = Dev(a.port)
    info = dict(fw=d.ask("FWOPT?"), freq=a.freq, gain=a.gain)
    print(info["fw"], d.ask(f"FREQ {int(a.freq)}") if float(a.freq).is_integer() else d.ask(f"FREQK {int(round(a.freq * 1000))}"))
    print(d.ask(f"GAIN MANUAL {a.gain}"))
    if a.dccal:
        print(d.ask("DCCAL", 10))
    if a.plsb == "auto":
        lv = d.ask("PLEVEL?").split(); lsb = int(lv[1]); info["plevel"] = " ".join(lv)
    else:
        lsb = int(a.plsb)
    print(d.ask(f"PLSB {lsb}"), "lsb", lsb)
    pc, pt = [], []
    for _ in range(a.pcap):
        b, t0, _l = d.pcap(); pc.append(b); pt.append(t0)
    cw, ct = [], []
    for _ in range(a.capt):
        w, t0 = d.capt20(); cw.append(w); ct.append(t0)
    print(d.ask("PLSB AUTO"))
    d.close()
    np.savez_compressed(a.out, pcap=np.array(pc), pcap_t0=np.array(pt), capt=np.array(cw), capt_t0=np.array(ct), lsb=lsb, info=str(info))
    print("saved", a.out, len(pc), "PCAP", len(cw), "CAPT20")


if __name__ == "__main__":
    main()
