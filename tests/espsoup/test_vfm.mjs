// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
// VFM host test: synthetic FM video (PAL/NTSC composite model)
// -> raw dump words (device orientation, 10-bit) -> demod_host.exe (the unchanged firmware DSP, block +
// gap simulation of the continuous engine) -> line records, compared with
//   (a) the truth picture and line numbers of the model,
//   (b) a floating-point reference demodulator (atan2 discriminator, same slicing windows).
// Run: DEMOD_HOST=<built demod_host> node test_vfm.mjs   (vectors, records and PNGs of the assembled frames go to
// ESPSOUP_TEST_DIR or a temporary directory; tests/test_espsoup_c5.py does all of this)
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import zlib from 'node:zlib';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const WORK = process.env.ESPSOUP_TEST_DIR || fs.mkdtempSync(path.join(os.tmpdir(), 'espsoup-demod-'));
const VEC = path.join(WORK, 'vec'), OUT = path.join(WORK, 'out');
fs.mkdirSync(VEC, { recursive: true }); fs.mkdirSync(OUT, { recursive: true });
const HOST = process.env.DEMOD_HOST || path.join(here, process.platform === 'win32' ? 'demod_host.exe' : 'demod_host');

function rng(seed) {
  let a = seed >>> 0;
  const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  let spare = null;
  u.gauss = () => { if (spare !== null) { const s = spare; spare = null; return s; } let x, y, r; do { x = 2 * u() - 1; y = 2 * u() - 1; r = x * x + y * y; } while (r >= 1 || r === 0); const f = Math.sqrt(-2 * Math.log(r) / r); spare = y * f; return x * f; };
  return u;
}
// ---- composite model: picture content as test_fpv.mjs, vertical interval per ITU-R BT.470 (interlaced,
// half-line equalising/broad pulses), line L = 1..625 (PAL) / 1..525 (NTSC), sync edge = line start ----
const STD = {
  PAL: { T: 64e-6, lines: 625, act0: 10.5e-6, actL: 52e-6, f1: [23, 310], f2: [336, 623] },
  NTSC: { T: 1 / 15734.264, lines: 525, act0: 9.4e-6, actL: 52.6e-6, f1: [22, 261], f2: [285, 524] },
};
// half-line pulse pattern [first half, second half]: B broad, E equalising, S normal sync, '-' nothing
function halves(std, L) {
  if (std === 'PAL') {
    if (L <= 2 || L === 314 || L === 315) return 'BB';
    if (L === 3) return 'BE';
    if (L === 313) return 'EB';
    if (L === 4 || L === 5 || L === 311 || L === 312 || L === 316 || L === 317 || L === 624 || L === 625) return 'EE';
    if (L === 318) return 'E-';
    if (L === 623) return 'SE';
    return 'S-';
  }
  if (L <= 3 || (L >= 7 && L <= 9) || L === 264 || L === 265 || L === 270 || L === 271) return 'EE';
  if (L >= 4 && L <= 6) return 'BB';
  if (L === 267 || L === 268) return 'BB';
  if (L === 266) return 'EB';
  if (L === 269) return 'BE';
  if (L === 272) return 'E-';
  if (L === 263) return 'SE';
  return 'S-';
}
const BARS = [1.0, 0.886, 0.701, 0.587, 0.413, 0.299, 0.114, 0.0].map(v => v * 0.7);
function picture(rows, row, u) {
  let v = row < rows * 2 / 3 ? BARS[Math.min(7, Math.floor(u * 8))] : 0.7 * u;
  const ud = (row % rows) / rows;
  if (Math.abs(u - ud) < 0.012) v = v > 0.35 ? 0.0 : 0.7;
  return v;
}
function activeRow(st, L) {
  if (L >= st.f1[0] && L <= st.f1[1]) return { row: L - st.f1[0], rows: st.f1[1] - st.f1[0] + 1 };
  if (L >= st.f2[0] && L <= st.f2[1]) return { row: L - st.f2[0], rows: st.f2[1] - st.f2[0] + 1 };
  return null;
}
function composite(stName, t) {
  const st = STD[stName], frame = st.lines * st.T;
  t = ((t % frame) + frame) % frame;
  const idx = Math.floor(t / st.T), L = idx + 1, tl = t - idx * st.T, half = st.T / 2;
  const hv = halves(stName, L), h = tl < half ? 0 : 1, th = tl - h * half, c = hv[h];
  if (c === 'B') return th < half - 4.7e-6 ? -0.3 : 0;
  if (c === 'E') return th < 2.35e-6 ? -0.3 : 0;
  if (c === '-' && hv[0] !== 'S') return 0;
  if (tl < 4.7e-6) return -0.3;
  const ar = activeRow(st, L);
  if (!ar || tl < st.act0 || tl >= st.act0 + st.actL) return 0;
  return picture(ar.rows, ar.row, (tl - st.act0) / st.actL);
}

// ---- generator: FM video -> device dump words ----
function genVideoWords({ std = 'PAL', fs = 20e6, seconds = 0.1, off = 1e6, dev = 3e6, snrDb = 25, seed = 1, polarity = 1, amp = 0.35, T0 = 0.0123, ppm = 0 }) {
  const st = STD[std], r = rng(seed), N = Math.round(seconds * fs), kc = 1 + ppm * 1e-6;   // kc: transmitter clock vs receiver
  const sig = 0.05e-6 * fs, H = Math.ceil(4 * sig), taps = [];
  let tsum = 0; for (let k = -H; k <= H; k++) { const w = Math.exp(-0.5 * (k / sig) ** 2); taps.push(w); tsum += w; }
  let vref = 0; const nref = 6400;
  for (let i = 0; i < nref; i++) vref += composite(std, (st.f1[0] + 9) * st.T + i / nref * st.T);
  vref /= nref;
  const k = 2 * dev, words = new Uint32Array(N), iq = new Float32Array(2 * N);
  const ring = new Float64Array(taps.length);
  for (let j = 0; j < taps.length - 1; j++) ring[j] = composite(std, (T0 + (j - H) / fs) * kc);
  let ph = r() * 2 * Math.PI;
  const nsd = amp * 10 ** (-snrDb / 20) / Math.SQRT2;
  for (let n = 0; n < N; n++) {
    ring[(n + taps.length - 1) % taps.length] = composite(std, (T0 + (n + H) / fs) * kc);
    let v = 0; for (let j = 0; j < taps.length; j++) v += taps[j] * ring[(n + j) % taps.length];
    v /= tsum;
    ph += 2 * Math.PI * (off + polarity * k * (v - vref)) / fs;
    const I = amp * Math.cos(ph) + nsd * r.gauss(), Q = amp * Math.sin(ph) + nsd * r.gauss();
    const qi = Math.max(-512, Math.min(511, Math.round(I * 512))), qq = Math.max(-512, Math.min(511, Math.round(-Q * 512)));   // device Q = -host Q
    iq[2 * n] = qi / 512; iq[2 * n + 1] = -qq / 512;
    words[n] = ((qi & 1023) | ((qq & 1023) << 10) | (((n * 2654435761) >>> 0) & 0xfff00000)) >>> 0;  // junk status bits
  }
  const fTip = off + polarity * k * (-0.3 - vref), fBlank = off + polarity * k * (0 - vref);
  return { words, iq, fs, N, st, T0, fTip, fBlank, std, kc };
}

// ---- harness + parse ----
function runHost(mode, file, outFile, args) {
  const r = execFileSync(HOST, [mode, file, outFile, ...args.map(String)], { encoding: 'utf8' });
  return r.trim();
}
function parseLines(buf, px) {
  const rec = 4 + 8 + px, out = [];
  for (let o = 0; o + rec <= buf.length; o += rec) {
    out.push({ blk: buf.readUInt32LE(o), line: buf.readUInt16LE(o + 4), flags: buf[o + 6], fc: buf[o + 7], t16: buf.readUInt32LE(o + 8), px: buf.subarray(o + 12, o + rec) });
  }
  return out;
}
// float reference: atan2 discriminator, box average over the same windows, ideal levels
function refLine(g, tEdge, px, full) {
  const st = g.st, a0 = full ? 0 : st.act0, L = full ? st.T : st.actL, out = new Float64Array(px);
  for (let p = 0; p < px; p++) {
    const s0 = (tEdge + a0 + p * L / px) * g.fs, s1 = (tEdge + a0 + (p + 1) * L / px) * g.fs;
    let sum = 0, cnt = 0;
    for (let n = Math.max(1, Math.ceil(s0)); n < s1 && n < g.N; n++) {
      const I1 = g.iq[2 * n], Q1 = g.iq[2 * n + 1], I0 = g.iq[2 * n - 2], Q0 = g.iq[2 * n - 1];
      sum += Math.atan2(Q1 * I0 - I1 * Q0, I1 * I0 + Q1 * Q0) * g.fs / (2 * Math.PI); cnt++;
    }
    const f = cnt ? sum / cnt : 0;
    out[p] = 64 + 64 * (f - g.fBlank) / (g.fBlank - g.fTip);
  }
  return out;
}
function truthLine(g, tAbs, px, full) {   // tAbs = absolute model time of the sync edge; line = 1-based number
  const st = g.st, frame = st.lines * st.T, tt = ((tAbs % frame) + frame) % frame;
  const idx = Math.round(tt / st.T) % st.lines, out = new Float64Array(px);
  const a0 = full ? 0 : st.act0, L = full ? st.T : st.actL;
  for (let p = 0; p < px; p++) {
    let s = 0; for (let q = 0; q < 8; q++) s += composite(g.std, idx * st.T + a0 + (p + (q + 0.5) / 8) * L / px);
    out[p] = 64 + 64 * (s / 8) / 0.3;
  }
  return { line: idx + 1, px: out };
}

// ---- PNG ----
const CRC = Uint32Array.from({ length: 256 }, (_, i) => { let c = i; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; return c >>> 0; });
const crc = b => { let c = 0xffffffff; for (const x of b) c = CRC[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
const chunk = (t, d) => { const l = Buffer.alloc(4); l.writeUInt32BE(d.length); const td = Buffer.concat([Buffer.from(t), d]), c = Buffer.alloc(4); c.writeUInt32BE(crc(td)); return Buffer.concat([l, td, c]); };
function writeGray(file, w, h, data) {
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 0;
  const raw = Buffer.alloc((w + 1) * h); for (let y = 0; y < h; y++) Buffer.from(data.buffer, data.byteOffset + y * w, w).copy(raw, y * (w + 1) + 1);
  fs.writeFileSync(file, Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]));
}

let failures = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`); if (!ok) failures++; };

// PARLIO source (parlio_cap.h): 40 MS/s bytes, low nibble = I[lsb+3:lsb], high nibble = Q (device orientation), with an
// optional DC offset before the 4-bit window (measured on hardware: ~-17 LSB); wrap-around like the hardware window.
function toParlioBytes(g, lsb, dcI = 0, dcQ = 0) {
  const b = new Uint8Array(g.N + ((4 - g.N % 4) % 4));
  for (let n = 0; n < g.N; n++) {
    const w = g.words[n]; let qi = w & 1023, qq = (w >> 10) & 1023; if (qi >= 512) qi -= 1024; if (qq >= 512) qq -= 1024;
    const i4 = Math.floor((qi + dcI) / (1 << lsb)) & 15, q4 = Math.floor((qq + dcQ) / (1 << lsb)) & 15;
    b[n] = i4 | (q4 << 4);
  }
  return b;
}
function scenario(name, gopt, { words = 8192, gapUs = 89, D = 0, px = 128, stdF = 0, polF = 0, sel = 1, rate = 0, expectStd, minLinesPerSec = 0, p8 = null } = {}) {
  const g = genVideoWords(gopt), fin = g.fs, gap = Math.round(gapUs * 1e-6 * fin);
  const vf = path.join(VEC, `${name}.${p8 ? 'u8' : 'u32'}`), of = path.join(OUT, `${name}.bin`);
  fs.writeFileSync(vf, p8 ? Buffer.from(toParlioBytes(g, p8.lsb, p8.dcI, p8.dcQ)) : Buffer.from(g.words.buffer));
  const stat = runHost(p8 ? 'vfmp' : 'vfm', vf, of, [fin, words, gap, D, px, stdF, polF, sel, rate]);
  const lines = parseLines(fs.readFileSync(of), px), full = !!(sel & 2);
  const st = Object.fromEntries(stat.split(' ').slice(1).reduce((a, x, i, arr) => (i % 2 ? a : [...a, [x, arr[i + 1]]]), []));
  // compare
  let eT = 0, eR = 0, eRT = 0, n = 0, nA = 0, worst = 0, edgeErr = 0, edgeMax = 0, lnBad = 0, lnN = 0, lnOff = null;
  const B = fin / 5e6;
  for (const l of lines) {
    const start = l.blk * (words + gap), t0us = Math.floor(start * 1e6 / fin);   // block t0 is whole us (like esp_timer)
    const tEdge = p8 ? l.t16 / 16 * 1e-6                                          // PARLIO: t16 counts exactly from byte 0
      : (l.t16 / 16 - t0us) * 1e-6 + start / fin;                                // seconds since file start
    const tAbs = (g.T0 + tEdge) * g.kc, tr = truthLine(g, tAbs, px, full), rf = refLine(g, tEdge, px, full);
    // sync edge vs truth grid (model sync edge = line start, 50 % point)
    const frac = (tAbs / g.st.T) - Math.round(tAbs / g.st.T);
    const ee = Math.abs(frac * g.st.T * 1e9); edgeErr += ee; edgeMax = Math.max(edgeMax, ee);
    const active = !!activeRow(g.st, tr.line) && halves(g.std, tr.line) === 'S-';
    for (let p = 2; p < px - 2; p++) {
      const v = l.px[p];
      eR += Math.abs(v - Math.max(0, Math.min(255, rf[p]))); eRT += Math.abs(Math.max(0, Math.min(255, rf[p])) - tr.px[p]);
      if (active) { const e = Math.abs(v - tr.px[p]); eT += e; nA++; worst = Math.max(worst, e); }
      n++;
    }
    if (l.flags & 8) {                                 // line number valid: model line index -> standard number
      const ln = l.line; lnN++;
      const off = (ln - tr.line + 10 * g.st.lines) % g.st.lines;       // 0 expected
      if (lnOff === null) lnOff = off; else if (off !== lnOff) lnBad++;
    }
  }
  const dur = g.N / fin, lps = lines.length / dur;
  console.log(`  ${name}: ${stat}`);
  console.log(`  ${name}: ${lines.length} lines out (${lps.toFixed(0)}/s), MAE fw-vs-truth ${(eT / Math.max(1, nA)).toFixed(2)} (active, max ${worst.toFixed(0)}), fw-vs-float-ref ${(eR / Math.max(1, n)).toFixed(2)}, float-ref-vs-truth ${(eRT / Math.max(1, n)).toFixed(2)} counts; edge err mean ${(edgeErr / Math.max(1, lines.length)).toFixed(1)} ns max ${edgeMax.toFixed(0)} ns; line-number offset ${lnOff} (inconsistent ${lnBad}/${lnN})`);
  // picture: place lines by line number into a frame (interlaced) or by time
  if (lines.length) {
        // interlaced frame: field 1 rows even, field 2 rows odd (st.f1/f2 active ranges)
    const rows = 2 * (g.st.f1[1] - g.st.f1[0] + 1), fr = new Uint8Array(px * rows);
    for (const l of lines) if (l.flags & 8) {
      const a = activeRow(g.st, l.line); if (!a) continue;
      const y = 2 * a.row + (l.line >= g.st.f2[0] ? 1 : 0); for (let p = 0; p < px; p++) fr[y * px + p] = l.px[p];
    }
    writeGray(path.join(OUT, `${name}.png`), px, rows, fr);
  }
  return { st, lines, mae: eT / Math.max(1, nA), maeRef: eR / Math.max(1, n), edgeErr: edgeErr / Math.max(1, lines.length), lnOff, lnBad, lps, dur, g };
}

console.log('VFM host tests (firmware DSP compiled for the host)');
// 1. PAL 20 MS/s, D=2 (default: phase of every 2nd sample), 128 px, 0.12 s, gaps 89 us
{
  const r = scenario('pal20', { std: 'PAL', fs: 20e6, seconds: 0.12, off: 1e6, dev: 3e6, snrDb: 25, seed: 11 }, { D: 0, px: 128, sel: 1 });
  check('PAL 20 MS/s: locked as PAL', r.st.std === '1' && r.lines.length > 0, `std ${r.st.std}`);
  check('PAL 20 MS/s: luminance MAE vs truth < 6 counts (of 149)', r.mae < 6, r.mae.toFixed(2));
  check('PAL 20 MS/s: fw vs float reference MAE < 5 counts', r.maeRef < 5, r.maeRef.toFixed(2));
  check('PAL 20 MS/s: sync edge error < 60 ns', r.edgeErr < 60, r.edgeErr.toFixed(1));
  check('PAL 20 MS/s: vertical sync found, line numbers exact', r.lnOff === 0 && r.lnBad === 0, `off ${r.lnOff} bad ${r.lnBad}`);
  check('PAL 20 MS/s: >= 9000 active lines/s (no USB limit)', r.lps > 9000, r.lps.toFixed(0));
}
// 1b. noiseless: firmware fixed point vs float reference (same windows) -> algorithmic error only
for (const [nm, fsr, D, dev] of [['pal20_clean_d2', 20e6, 2, 3e6], ['pal20_clean_d1', 20e6, 1, 3e6], ['pal20_clean_d4', 20e6, 4, 1.4e6], ['pal10_clean_d1', 10e6, 1, 2e6]]) {
  const r = scenario(nm, { std: 'PAL', fs: fsr, seconds: 0.06, off: 0.3e6, dev, snrDb: 60, seed: 21 }, { D, px: 128, sel: 1 });
  check(`${nm}: fw vs float reference MAE < ${D === 4 ? 3.5 : 2.5} counts`, r.maeRef < (D === 4 ? 3.5 : 2.5), r.maeRef.toFixed(2));
  check(`${nm}: line numbers exact`, r.lnOff === 0 && r.lnBad === 0, `off ${r.lnOff} bad ${r.lnBad}`);
}
// 2. PAL 10 MS/s D=1, dev 2.5 MHz (fits +-5 MHz)
{
  const r = scenario('pal10_d1', { std: 'PAL', fs: 10e6, seconds: 0.12, off: 0.3e6, dev: 2.0e6, snrDb: 25, seed: 12 }, { D: 1, px: 160, sel: 1 });
  check('PAL 10 MS/s: MAE vs truth < 6', r.mae < 6, r.mae.toFixed(2));
  check('PAL 10 MS/s: fw vs float ref < 6', r.maeRef < 6, r.maeRef.toFixed(2));
  check('PAL 10 MS/s: line numbers exact', r.lnOff === 0 && r.lnBad === 0, `off ${r.lnOff} bad ${r.lnBad}`);
  check('PAL 10 MS/s: >= 11000 active lines/s', r.lps > 11000, r.lps.toFixed(0));
}
// 3. NTSC 20 MS/s, inverted polarity, D=2
{
  const r = scenario('ntsc20_inv', { std: 'NTSC', fs: 20e6, seconds: 0.1, off: -1.5e6, dev: 3e6, snrDb: 25, seed: 13, polarity: -1 }, { D: 2, px: 128, sel: 1 });
  check('NTSC 20 MS/s inverted: locked as NTSC', r.st.std === '2', `std ${r.st.std} pol ${r.st.pol}`);
  check('NTSC inverted: MAE vs truth < 6', r.mae < 6, r.mae.toFixed(2));
  check('NTSC inverted: line numbers exact', r.lnOff === 0 && r.lnBad === 0, `off ${r.lnOff} bad ${r.lnBad}`);
}
// 4. USB budget 850 kB/s: lines limited, evenly spread
{
  const r = scenario('pal20_rate', { std: 'PAL', fs: 20e6, seconds: 0.12, off: 1e6, dev: 3e6, snrDb: 25, seed: 14 }, { D: 0, px: 128, sel: 1, rate: 850000 });
  const bytesPerSec = r.lines.length * (8 + 128) / r.dur;
  check('rate limit: <= 850 kB/s of line records', bytesPerSec <= 850000 * 1.05 + 8192 / r.dur, (bytesPerSec / 1000).toFixed(0) + ' kB/s');
}
// 5. low SNR (noise in the full 20 MHz): 12 dB and 9 dB (FM threshold region)
{
  const r = scenario('pal20_snr12', { std: 'PAL', fs: 20e6, seconds: 0.1, off: 1e6, dev: 3e6, snrDb: 12, seed: 15 }, { D: 0, px: 128, sel: 1 });
  check('PAL 12 dB SNR: locked, MAE < 8, line numbers >= 98 % right', r.lines.length > 900 && r.mae < 8 && r.lnBad <= 0.02 * r.lines.length, `${r.lines.length} lines, MAE ${r.mae.toFixed(2)}, bad ${r.lnBad}`);
  const r9 = scenario('pal20_snr9', { std: 'PAL', fs: 20e6, seconds: 0.1, off: 1e6, dev: 3e6, snrDb: 9, seed: 16 }, { D: 0, px: 128, sel: 1 });
  check('PAL 9 dB SNR: locked, MAE < 15, line numbers exact', r9.lines.length > 800 && r9.mae < 15 && r9.lnOff === 0 && r9.lnBad === 0, `${r9.lines.length} lines, MAE ${r9.mae.toFixed(2)}, off ${r9.lnOff} bad ${r9.lnBad}`);
}
// 5b. 0.4 s with the transmitter clock +60 ppm off (line period tracking over 10 fields)
{
  const r = scenario('pal20_ppm', { std: 'PAL', fs: 20e6, seconds: 0.4, off: 0.8e6, dev: 3e6, snrDb: 20, seed: 17, ppm: 60 }, { D: 0, px: 128, sel: 1 });
  check('PAL +60 ppm, 0.4 s: MAE < 6, edge err < 60 ns, line numbers exact', r.mae < 6 && r.edgeErr < 60 && r.lnOff === 0 && r.lnBad === 0, `MAE ${r.mae.toFixed(2)} edge ${r.edgeErr.toFixed(1)} ns off ${r.lnOff} bad ${r.lnBad} T16 ${r.st.T16}`);
}
// 6. noise only: no lock
{
  const g = { words: new Uint32Array(20e6 * 0.03) }; const r = rng(99);
  for (let n = 0; n < g.words.length; n++) { const i = Math.round(r.gauss() * 60) & 1023, q = Math.round(r.gauss() * 60) & 1023; g.words[n] = i | (q << 10); }
  const vf = path.join(VEC, 'noise.u32'), of = path.join(OUT, 'noise.bin'); fs.writeFileSync(vf, Buffer.from(g.words.buffer));
  const stat = runHost('vfm', vf, of, [20e6, 8192, 1780, 0, 128, 0, 0, 1, 0]);
  check('noise: no lines', fs.readFileSync(of).length === 0, stat);
}
// 7. PARLIO source (vf_block_p8): 4+4-bit bytes at 40 MS/s, gap-free, 8192-byte blocks with overlap
{
  const r = scenario('p8_pal', { std: 'PAL', fs: 40e6, seconds: 0.2, off: 1e6, dev: 3e6, snrDb: 25, seed: 31 }, { words: 8192, gapUs: 0, px: 128, sel: 1, p8: { lsb: 5, dcI: -17, dcQ: 6 } });
  check('PARLIO PAL 4-bit: locked as PAL', r.st.std === '1' && r.lines.length > 0, `std ${r.st.std}`);
  check('PARLIO PAL 4-bit: luminance MAE vs truth < 8 counts', r.mae < 8, r.mae.toFixed(2));
  check('PARLIO PAL 4-bit: sync edge error < 80 ns', r.edgeErr < 80, r.edgeErr.toFixed(1));
  check('PARLIO PAL 4-bit: line numbers exact', r.lnOff === 0 && r.lnBad === 0, `off ${r.lnOff} bad ${r.lnBad}`);
  check('PARLIO PAL 4-bit: gap-free, >= 14000 active lines/s (all non-VBI lines: 14400/s)', r.lps >= 14000 && +r.st.drop_cut <= 2, `${r.lps.toFixed(0)} /s, drop_cut ${r.st.drop_cut}`);
  const all = scenario('p8_pal_all', { std: 'PAL', fs: 40e6, seconds: 0.2, off: 1e6, dev: 3e6, snrDb: 25, seed: 32 }, { words: 8192, gapUs: 0, px: 64, sel: 0, p8: { lsb: 5 } });
  check('PARLIO PAL incl. VBI: >= 15500 lines/s (15625 = every line)', all.lps >= 15500, `${all.lps.toFixed(0)} /s`);
  const nt = scenario('p8_ntsc_inv', { std: 'NTSC', fs: 40e6, seconds: 0.15, off: -1.5e6, dev: 3e6, snrDb: 25, seed: 33, polarity: -1 }, { words: 8192, gapUs: 0, px: 128, sel: 1, p8: { lsb: 5 } });
  check('PARLIO NTSC inverted: NTSC, MAE < 8, line numbers exact, >= 14000 lines/s', nt.st.std === '2' && nt.mae < 8 && nt.lnOff === 0 && nt.lnBad === 0 && nt.lps >= 14000,
    `std ${nt.st.std} MAE ${nt.mae.toFixed(2)} off ${nt.lnOff} bad ${nt.lnBad} ${nt.lps.toFixed(0)}/s`);
  const w = scenario('p8_pal_snr12', { std: 'PAL', fs: 40e6, seconds: 0.1, off: 1e6, dev: 3e6, snrDb: 12, seed: 34 }, { words: 8192, gapUs: 0, px: 128, sel: 1, p8: { lsb: 5 } });
  check('PARLIO PAL 12 dB SNR (40 MHz noise band): MAE < 12, line numbers exact', w.mae < 12 && w.lnOff === 0 && w.lnBad === 0, `MAE ${w.mae.toFixed(2)} off ${w.lnOff} bad ${w.lnBad}`);
  const ov = scenario('p8_pal_overrun', { std: 'PAL', fs: 40e6, seconds: 0.2, off: 1e6, dev: 3e6, snrDb: 25, seed: 35 }, { words: 8192, gapUs: 204.8, px: 128, sel: 1, p8: { lsb: 5 } });
  check('PARLIO with ring overruns (one 8192-byte block lost each time): line numbers still exact', ov.lnOff === 0 && ov.lnBad === 0 && ov.mae < 8, `MAE ${ov.mae.toFixed(2)} off ${ov.lnOff} bad ${ov.lnBad} ${ov.lps.toFixed(0)}/s`);
}
console.log(failures ? `${failures} FAILED` : 'all VFM tests passed');
process.exit(failures ? 1 : 0);
