// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
// VFM colour (VFMCOLOR) host test: synthetic colour composite (PAL with V switch and +-135 deg burst, NTSC with
// 180 deg burst, ITU vertical interval) -> FM -> PARLIO bytes (40 MS/s, 4+4 bit window, device orientation) ->
// demod_host vfmp with colour (the unchanged firmware DSP, bit-exact with the device) -> colour records, compared with
// the truth of the 75 % colour bars: hue, saturation, chroma SNR, PAL V switch, record count/selection, and the luma
// records must be byte-identical with and without colour.
// Run: DEMOD_HOST=<built demod_host> node test_vfm_color.mjs   (tests/test_espsoup_c5.py does this)
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const WORK = process.env.ESPSOUP_TEST_DIR || fs.mkdtempSync(path.join(os.tmpdir(), 'espsoup-color-'));
const VEC = path.join(WORK, 'cvec'); fs.mkdirSync(VEC, { recursive: true });
const HOST = process.env.DEMOD_HOST || path.join(here, process.platform === 'win32' ? 'demod_host.exe' : 'demod_host');

function rng(seed) {
  let a = seed >>> 0;
  const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  let spare = null;
  u.gauss = () => { if (spare !== null) { const s = spare; spare = null; return s; } let x, y, r; do { x = 2 * u() - 1; y = 2 * u() - 1; r = x * x + y * y; } while (r >= 1 || r === 0); const f = Math.sqrt(-2 * Math.log(r) / r); spare = y * f; return x * f; };
  return u;
}
const STD = {
  PAL: { T: 64e-6, lines: 625, act0: 10.5e-6, actL: 52e-6, f1: [23, 310], f2: [336, 623], fsc: 4433618.75, b0: 5.6e-6, bl: 2.25e-6, bA: 0.15, sync: 0.3, white: 0.7, setup: 0 },
  NTSC: { T: 1 / 15734.264, lines: 525, act0: 9.4e-6, actL: 52.6e-6, f1: [22, 261], f2: [285, 524], fsc: 315e6 / 88, b0: 5.3e-6, bl: 2.5e-6, bA: 0.143, sync: 0.286, white: 0.714, setup: 0.0536 },
};
function halves(std, L) {          // ITU-R BT.470 / SMPTE 170M half-line pulses (as test_vfm.mjs)
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
// 75 % EBU bars (R'G'B'), upper 45 % of each field; below: grey steps
const BARS = [[.75, .75, .75], [.75, .75, 0], [0, .75, .75], [0, .75, 0], [.75, 0, .75], [.75, 0, 0], [0, 0, .75], [0, 0, 0]];
const NAMES = ['white', 'yellow', 'cyan', 'green', 'magenta', 'red', 'blue', 'black'];
const yuv = ([r, g, b]) => { const y = 0.299 * r + 0.587 * g + 0.114 * b; return [y, 0.493 * (b - y), 0.877 * (r - y)]; };
function picture(rows, row, u) {
  if (row < rows * 0.45) return yuv(BARS[Math.min(7, Math.floor(u * 8))]);
  return [Math.min(5, Math.floor(u * 6)) / 5, 0, 0];
}
function activeRow(st, L) {
  if (L >= st.f1[0] && L <= st.f1[1]) return { row: L - st.f1[0], rows: st.f1[1] - st.f1[0] + 1 };
  if (L >= st.f2[0] && L <= st.f2[1]) return { row: L - st.f2[0], rows: st.f2[1] - st.f2[0] + 1 };
  return null;
}
function composite(stName, t) {
  const st = STD[stName], frame = st.lines * st.T, w = 2 * Math.PI * st.fsc;
  const tm = ((t % frame) + frame) % frame;
  const idx = Math.floor(tm / st.T), L = idx + 1, tl = tm - idx * st.T, half = st.T / 2;
  const hv = halves(stName, L), h = tl < half ? 0 : 1, th = tl - h * half, c = hv[h];
  if (c === 'B') return th < half - 4.7e-6 ? -st.sync : 0;
  if (c === 'E') return th < 2.35e-6 ? -st.sync : 0;
  if (c === '-' && hv[0] !== 'S') return 0;
  if (tl < 4.7e-6) return -st.sync;
  const vsw = stName === 'PAL' && (L & 1) ? -1 : 1;
  if (tl >= st.b0 && tl < st.b0 + st.bl) {        // burst, 0.25 us raised-cosine ramps
    const x = tl - st.b0, r = 0.25e-6, env = x < r ? 0.5 - 0.5 * Math.cos(Math.PI * x / r) : x > st.bl - r ? 0.5 - 0.5 * Math.cos(Math.PI * (st.bl - x) / r) : 1;
    return stName === 'PAL' ? st.bA * env * (-Math.sin(w * t) + vsw * Math.cos(w * t)) / Math.SQRT2 : -st.bA * env * Math.sin(w * t);
  }
  const ar = activeRow(st, L);
  if (!ar || tl < st.act0 || tl >= st.act0 + st.actL) return 0;
  const [Y, U, V] = picture(ar.rows, ar.row, (tl - st.act0) / st.actL), sc = st.white - st.setup;
  return st.setup + sc * Y + sc * (U * Math.sin(w * t) + vsw * V * Math.cos(w * t));
}
// FM at 40 MS/s (K Hz per volt around the mid level), noise, device orientation (Q negated), 4-bit window -> bytes.
// t is the transmitter's time (continuous subcarrier); the receiver starts at T0.
function genBytes({ std = 'PAL', seconds = 0.1, off = 0.5e6, K = 5e6, snrDb = 30, seed = 1, amp = 100, lsb = 4, T0 = 0.0213, ppm = 0 }) {
  const fs_ = 40e6, N = Math.round(seconds * fs_), r = rng(seed), b = new Uint8Array(N), kc = 1 + ppm * 1e-6;
  const nsd = amp * 10 ** (-snrDb / 20) / Math.SQRT2;
  let ph = r() * 2 * Math.PI;
  for (let n = 0; n < N; n++) {
    const v = composite(std, (T0 + n / fs_) * kc);
    ph += 2 * Math.PI * (off + K * (v - 0.2)) / fs_;
    const I = amp * Math.cos(ph) + nsd * r.gauss(), Q = -(amp * Math.sin(ph) + nsd * r.gauss());
    b[n] = (Math.floor(I / (1 << lsb)) & 15) | ((Math.floor(Q / (1 << lsb)) & 15) << 4);
  }
  return b;
}
function runHost(args) { return execFileSync(HOST, args.map(String), { encoding: 'utf8' }).trim(); }
function parseColor(buf, nseg) {
  const rec = 4 + 10 + 2 * nseg, out = [];
  for (let o = 0; o + rec <= buf.length; o += rec) {
    const uv = new Int8Array(buf.buffer, buf.byteOffset + o + 14, 2 * nseg);
    out.push({ blk: buf.readUInt32LE(o), line: buf.readUInt16LE(o + 4), flags: buf[o + 6], burst: buf[o + 7], t16: buf.readUInt32LE(o + 8), bph: buf[o + 12], rph: buf[o + 13],
               U: Array.from(uv.filter((_, i) => !(i & 1))), V: Array.from(uv.filter((_, i) => i & 1)) });
  }
  return out;
}
let failures = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`); if (!ok) failures++; };

// bar statistics: segments whose span lies inside one bar with a margin of a quarter bar
function barStats(cols, std, nseg) {
  const st = STD[std], scale = (st.white - st.setup) * 64 / st.sync, res = [];
  for (let k = 0; k < 8; k++) {
    const segs = []; for (let j = 0; j < nseg; j++) { const a = j / nseg, b = (j + 1) / nseg; if (a >= (k + 0.2) / 8 && b <= (k + 0.8) / 8) segs.push(j); }
    const U = [], V = [];
    for (const c of cols) for (const j of segs) { U.push(c.U[j]); V.push(c.V[j]); }
    const mu = U.reduce((x, y) => x + y, 0) / U.length, mv = V.reduce((x, y) => x + y, 0) / V.length;
    const sd = Math.sqrt((U.reduce((x, y) => x + (y - mu) ** 2, 0) + V.reduce((x, y) => x + (y - mv) ** 2, 0)) / (2 * U.length));
    const [, tu, tv] = yuv(BARS[k]), TU = tu * scale, TV = tv * scale, amp = Math.hypot(TU, TV);
    let hue = null; if (amp > 0) { hue = (Math.atan2(mv, mu) - Math.atan2(TV, TU)) * 180 / Math.PI; hue = ((hue + 540) % 360) - 180; }
    res.push({ bar: NAMES[k], U: +mu.toFixed(1), V: +mv.toFixed(1), TU: +TU.toFixed(1), TV: +TV.toFixed(1), noise: +sd.toFixed(2), sat: amp ? +(Math.hypot(mu, mv) / amp).toFixed(3) : null,
               hue: hue === null ? null : +hue.toFixed(1), snr: amp && sd ? +(20 * Math.log10(amp / sd)).toFixed(1) : null, n: U.length });
  }
  return res;
}
function scenario(name, gopt, { px = 64, nseg = 32, every = 1, step = 1, words = 8192 } = {}) {
  const std = gopt.std || 'PAL', st = STD[std];
  const bytes = genBytes(gopt), vf = path.join(VEC, `${name}.u8`);
  fs.writeFileSync(vf, Buffer.from(bytes.buffer, 0, bytes.length - bytes.length % 4));
  const lumaA = path.join(VEC, `${name}_l0.bin`), lumaB = path.join(VEC, `${name}_l1.bin`), colf = path.join(VEC, `${name}_c.bin`);
  runHost(['vfmp', vf, lumaA, 0, words, 0, 0, px, 0, 0, 1, 0]);
  const stat = runHost(['vfmp', vf, lumaB, 0, words, 0, 0, px, 0, 0, 1, 0, nseg, every, step, colf]);
  const same = Buffer.compare(fs.readFileSync(lumaA), fs.readFileSync(lumaB)) === 0;
  const cols = parseColor(fs.readFileSync(colf), nseg);
  const vfc = (stat.match(/VFC .*/) || [''])[0];
  const lines = +(stat.match(/VFM blocks \d+ lines (\d+)/) || [0, 0])[1];
  // bar lines: rows 3 .. 0.4 of each field (VLOCK records only), skip the first 20 ms (acquisition)
  const rows = st.f1[1] - st.f1[0] + 1, t16min = 0.02 * 16e6;
  const bar = cols.filter(c => (c.flags & 8) && c.t16 > t16min && ((c.line >= st.f1[0] + 3 && c.line <= st.f1[0] + 0.4 * rows) || (c.line >= st.f2[0] + 3 && c.line <= st.f2[0] + 0.4 * rows)));
  return { name, std, same, cols, bar, stats: barStats(bar, std, nseg), vfc, lines, stat };
}
function summary(r) {
  const s = r.stats.filter(x => x.sat !== null);
  const hueMax = Math.max(...s.map(x => Math.abs(x.hue))), satMin = Math.min(...s.map(x => x.sat)), satMax = Math.max(...s.map(x => x.sat));
  const snr = s.reduce((a, x) => a + x.snr, 0) / s.length;
  return { hueMax: +hueMax.toFixed(1), satMin, satMax, snr: +snr.toFixed(1) };
}

// 1. PAL, every line, step 1, nseg 32: colour bars within 3 deg / 6 %, V switch alternates, luma unchanged
{
  const r = scenario('pal_s1', { std: 'PAL', seconds: 0.12, snrDb: 30, seed: 11 }, { nseg: 32, every: 1, step: 1 });
  const m = summary(r);
  check('PAL luma records identical with colour on', r.same);
  check('PAL colour records for the active lines', r.cols.length > 0.8 * r.lines * 576 / 625, `${r.cols.length} of ${r.lines} lines; ${r.vfc}`);
  check('PAL bars hue error < 3 deg', m.hueMax < 3, JSON.stringify(m));
  check('PAL bars saturation 0.94..1.06', m.satMin > 0.94 && m.satMax < 1.06, JSON.stringify(m));
  check('PAL chroma SNR > 20 dB (SNR 30 dB input)', m.snr > 20, `${m.snr} dB`);
  const alt = r.bar.slice(1).filter((c, i) => (c.flags & 1) !== (r.bar[i].flags & 1) && c.line === r.bar[i].line + 1).length;
  const pairs = r.bar.slice(1).filter((c, i) => c.line === r.bar[i].line + 1).length;
  check('PAL V switch alternates line by line', pairs > 50 && alt / pairs > 0.99, `${alt}/${pairs}`);
  // the decoded V switch must follow the transmitter (V = +1 on even ITU lines in this model)
  const ok = r.bar.filter(c => ((c.flags & 1) ? 1 : 0) === (c.line & 1)).length;
  check('PAL V switch matches the transmitted phase', ok / r.bar.length > 0.99, `${ok}/${r.bar.length}`);
  const bm = r.bar.map(c => c.burst).sort((a, b) => a - b)[r.bar.length >> 1];
  check('PAL burst amplitude ~ 32 steps (0.15 V)', bm >= 28 && bm <= 36, `median ${bm}`);
  for (const x of r.stats) console.log('    ', JSON.stringify(x));
}
// 2. PAL steps 2 and 4, nseg 16, every 4 (selection rotates through the frame)
for (const step of [2, 4]) {
  const r = scenario(`pal_s${step}`, { std: 'PAL', seconds: 0.2, snrDb: 30, seed: 20 + step }, { nseg: 32, every: 4, step });
  const m = summary(r);
  check(`PAL step ${step}: luma unchanged`, r.same);
  check(`PAL step ${step}: hue < 4 deg, saturation 0.92..1.08`, m.hueMax < 4 && m.satMin > 0.92 && m.satMax < 1.08, JSON.stringify(m));
  // every 4: consecutive colour records 4 lines apart, and the line set moves from frame to frame
  const d = r.cols.slice(1).map((c, i) => (c.line - r.cols[i].line + 625) % 625).filter(x => x > 0 && x < 20);
  const frac4 = d.filter(x => x === 4).length / d.length;
  check(`PAL step ${step}: every 4th line`, frac4 > 0.9, `${(100 * frac4).toFixed(1)} % spaced 4`);
  const f = {}; for (const c of r.cols) if (c.flags & 8) (f[c.line % 4] = (f[c.line % 4] || 0) + 1);
  check(`PAL step ${step}: all line residues mod 4 get colour (rotation)`, Object.keys(f).length === 4, JSON.stringify(f));
}
// 3. NTSC, step 1 and 4
for (const step of [1, 4]) {
  const r = scenario(`ntsc_s${step}`, { std: 'NTSC', seconds: 0.12, snrDb: 30, seed: 40 + step }, { nseg: 32, every: 1, step });
  const m = summary(r);
  check(`NTSC step ${step}: luma unchanged`, r.same);
  check(`NTSC step ${step}: hue < 4 deg, saturation 0.92..1.08`, m.hueMax < 4 && m.satMin > 0.92 && m.satMax < 1.08, JSON.stringify(m));
  check(`NTSC step ${step}: no V switch`, r.bar.every(c => !(c.flags & 1)) && r.bar.every(c => c.flags & 32), `${r.bar.length} lines`);
}
// 4. noisy PAL (12 dB) and a transmitter clock offset (+40 ppm: subcarrier phase drifts along the line)
{
  const r = scenario('pal_snr18', { std: 'PAL', seconds: 0.12, snrDb: 18, seed: 51 }, { nseg: 32, every: 1, step: 2 });
  const m = summary(r);
  check('PAL 18 dB: colour still decoded (hue < 10 deg)', m.hueMax < 10, JSON.stringify(m) + ` ${r.bar.length} bar lines`);
  const p = scenario('pal_ppm', { std: 'PAL', seconds: 0.12, snrDb: 30, seed: 52, ppm: 40 }, { nseg: 32, every: 1, step: 1 });
  const mp = summary(p);
  check('PAL +40 ppm: hue < 4 deg', mp.hueMax < 4, JSON.stringify(mp));
}
// 5. bad configurations are rejected by the harness (vc_init)
{
  let rej = 0;
  for (const bad of [[3, 1, 1], [65, 1, 1], [16, 0, 1], [16, 1, 3]]) {
    try { runHost(['vfmp', path.join(VEC, 'pal_s1.u8'), path.join(VEC, 'x.bin'), 0, 8192, 0, 0, 64, 0, 0, 1, 0, ...bad, path.join(VEC, 'xc.bin')]); } catch { rej++; }
  }
  check('invalid colour configurations rejected', rej === 4, `${rej}/4`);
}
console.log(failures ? `${failures} FAILED` : 'all colour tests passed');
process.exit(failures ? 1 : 0);
