// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
// GFSK host test: synthetic packet streams (BLE advertising with whitening/CRC-24, nRF24-style
// 2 Mbit GFSK with a sync word, CC2500-style 250 kbit 2-FSK, preamble-only trigger) -> raw dump words
// -> demod_host.exe (firmware DSP, engine blocks + gaps) -> packets compared with the transmitted bytes.
// Optional float reference: set ESPSOUP_BLE_JS to a ble.js that defines bleDecode(iq, fs, ch) (e.g. the ESPsoup
// app's snapshot decoder); the firmware must decode at least as many packets on the same blocks.
// Run: DEMOD_HOST=<built demod_host> node test_gfsk.mjs   (tests/test_espsoup_c5.py does this)
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import vm from 'node:vm';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
// BLE link layer (Core spec Vol 6 Part B 3.1/3.2): LSB-first bits, whitening x^7+x^4+1 seeded with
// 1 || channel, CRC-24 x^24+x^10+x^9+x^6+x^4+x^3+x+1 (advertising init 0x555555, sent MSB first).
const bleBits = (v, n) => Array.from({ length: n }, (_, i) => (v >>> i) & 1);
function bleWhiten(bits, ch) {
  let lfsr = 0x40 | (ch & 0x3f);                      // bit 6 = position 0 of the spec register
  return bits.map(b => { const o = lfsr & 1; lfsr >>= 1; if (o) lfsr ^= 0x44; return b ^ o; });
}
function bleCrc(bits) {
  let c = 0x555555;
  for (const b of bits) { const fb = ((c >>> 23) & 1) ^ b; c = (c << 1) & 0xffffff; if (fb) c ^= 0x00065b; }
  return Array.from({ length: 24 }, (_, i) => (c >>> (23 - i)) & 1);
}
const bleDecode = process.env.ESPSOUP_BLE_JS
  ? vm.runInThisContext(fs.readFileSync(process.env.ESPSOUP_BLE_JS, 'utf8') + '\n;bleDecode', { filename: 'ble.js' }) : null;
const BLE_AA_ = 0x8e89bed6;
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
const msb = (v, n) => Array.from({ length: n }, (_, i) => (v >>> (n - 1 - i)) & 1);

// stream of packets: each {bits (air order), rate, dev, cfo, amp}; Gaussian BT (0 = plain 2-FSK)
function genStream({ fs, seconds, packets, bt = 0.5, noise = 0.02, dc = [0.03, -0.02], seed = 1 }) {
  const r = rng(seed), N = Math.round(seconds * fs), I = new Float64Array(N), Q = new Float64Array(N);
  const truth = [];
  let t = Math.round(200e-6 * fs);
  for (const p of packets) {
    const sps = fs / p.rate, nb = p.bits.length, len = Math.round(nb * sps), start = t;
    if (start + len + 64 >= N) break;
    // frequency pulse train, Gaussian filtered
    const f = new Float64Array(len + 1);
    for (let j = 0; j < len; j++) f[j] = p.bits[Math.min(nb - 1, Math.floor(j / sps))] * 2 - 1;
    let g = f;
    if (bt) {
      const sig = Math.sqrt(Math.log(2)) / (2 * Math.PI * bt) * sps, gl = Math.ceil(3 * sig), taps = [];
      let s = 0; for (let k = -gl; k <= gl; k++) { const w = Math.exp(-0.5 * (k / sig) ** 2); taps.push(w); s += w; }
      g = new Float64Array(len);
      for (let j = 0; j < len; j++) { let a = 0; for (let k = -gl; k <= gl; k++) { const q = Math.min(len - 1, Math.max(0, j + k)); a += taps[k + gl] * f[q]; } g[j] = a / s; }
    }
    let ph = r() * 6.283;
    for (let j = 0; j < len; j++) {
      ph += 2 * Math.PI * (g[j] * p.dev + p.cfo) / fs;
      const ramp = Math.min(1, j / (0.5 * sps), (len - j) / (0.5 * sps));
      I[start + j] += p.amp * ramp * Math.cos(ph); Q[start + j] += p.amp * ramp * Math.sin(ph);
    }
    truth.push({ ...p, start, end: start + len });
    t = start + len + Math.round((p.gapUs ?? 150 + r() * 900) * 1e-6 * fs);
  }
  const words = new Uint32Array(N), iq = new Float32Array(2 * N);
  for (let n = 0; n < N; n++) {
    const a = I[n] + noise * r.gauss() + dc[0], b = Q[n] + noise * r.gauss() + dc[1];
    const qi = Math.max(-512, Math.min(511, Math.round(a * 512))), qq = Math.max(-512, Math.min(511, Math.round(-b * 512)));
    iq[2 * n] = qi / 512; iq[2 * n + 1] = -qq / 512;
    words[n] = ((qi & 1023) | ((qq & 1023) << 10) | (((n * 2654435761) >>> 0) & 0xfff00000)) >>> 0;
  }
  return { words, iq, N, fs, truth };
}
function runGfsk(name, g, { words = 8192, gapUs = 89, R, sps, mode, len, sync = '0', nbits = 32, maxerr = 1, chan = 37, okonly = 0 }) {
  const vf = path.join(VEC, `${name}.u32`), of = path.join(OUT, `${name}.bin`), gap = Math.round(gapUs * 1e-6 * g.fs);
  fs.writeFileSync(vf, Buffer.from(g.words.buffer));
  const stat = execFileSync(HOST, ['gfsk', vf, of, g.fs, words, gap, R, sps, mode, len, sync, nbits, maxerr, chan, okonly].map(String), { encoding: 'utf8' }).trim();
  const b = fs.readFileSync(of), pk = [];
  for (let o = 0; o < b.length;) {
    const n = b.readUInt16LE(o + 8);
    pk.push({ blk: b.readUInt32LE(o), pos: b.readUInt32LE(o + 4), n, flags: b[o + 10], errs: b[o + 11], rssi: b.readInt16LE(o + 12) / 10, cfo: b.readInt32LE(o + 14), dev: b.readInt32LE(o + 18), data: b.subarray(o + 22, o + 22 + n) });
    o += 22 + n;
  }
  // which truth packets lie completely inside a block?
  const blkLen = words + gap;
  const inside = p => Math.floor(p.start / blkLen) === Math.floor((p.end - 1) / blkLen) && (p.end - 1) % blkLen < words && p.start % blkLen < words;
  return { stat, pk, inside, gap, words };
}

let failures = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  -- ' + detail : ''}`); if (!ok) failures++; };

// ---------------- 1. BLE advertising, 8 MS/s, R=2 (4 MS/s, 4 sps) and 4 MS/s R=1 ----------------
function blePackets(r, n, { cfoMax = 100e3, ampLo = 0.08, ampHi = 0.4 } = {}) {
  const out = [];
  for (let k = 0; k < n; k++) {
    const adLen = 3 + Math.floor(r() * 29), pdu = [0x42, 6 + adLen];
    for (let j = 0; j < 6 + adLen; j++) pdu.push(Math.floor(r() * 256));
    let bits = []; for (const x of pdu) bits.push(...bleBits(x, 8));
    bits = [...bits, ...bleCrc(bits)];
    const air = [...bleBits(0xaa, 8), ...bleBits(BLE_AA_, 32), ...bleWhiten(bits, 37)];
    out.push({ bits: air, rate: 1e6, dev: 250e3, cfo: (2 * r() - 1) * cfoMax, amp: ampLo + r() * (ampHi - ampLo), pdu: Uint8Array.from(pdu) });
  }
  return out;
}
for (const [name, fsr, R, sps] of [['ble8', 8e6, 2, 4], ['ble4', 4e6, 1, 4], ['ble8r1', 8e6, 1, 8]]) {
  const r = rng(fsr / 1e5 + R), pk = blePackets(r, 300);
  const g = genStream({ fs: fsr, seconds: 0.45, packets: pk, seed: 7 + R });
  const res = runGfsk(name, g, { R, sps, mode: 0, len: 37, sync: '8E89BED6', chan: 37 });
  let exp = 0, got = 0, cfoErr = 0, fp = 0;
  const used = new Set();
  for (const t of g.truth) {
    if (!res.inside(t)) continue; exp++;
    const m = res.pk.find(p => (p.flags & 1) && Math.abs(p.pos - (t.start + 8 * fsr / 1e6)) < 4 * fsr / 1e6 && Buffer.compare(Buffer.from(p.data.subarray(0, t.pdu.length)), Buffer.from(t.pdu)) === 0);
    if (m) { got++; used.add(m); cfoErr += Math.abs(m.cfo - t.cfo); }
  }
  for (const p of res.pk) if ((p.flags & 1) && !used.has(p) && !g.truth.some(t => Buffer.compare(Buffer.from(p.data.subarray(0, t.pdu.length)), Buffer.from(t.pdu)) === 0)) fp++;
  // float reference (app decoder) per block
  let ref = 0; const bl = res.words + res.gap;
  if (bleDecode) for (let b = 0; b * bl + res.words <= g.N; b++) ref += bleDecode(g.iq.subarray(2 * b * bl, 2 * (b * bl + res.words)), fsr, 37).length;
  console.log(`  ${name}: ${res.stat}`);
  console.log(`  ${name}: ${g.truth.length} sent, ${exp} completely inside blocks, ${got} decoded with CRC + exact bytes (${(100 * got / exp).toFixed(1)} %), float reference ${bleDecode ? ref : "n/a"}, false CRC-ok ${fp}, mean |CFO err| ${(cfoErr / Math.max(1, got) / 1e3).toFixed(1)} kHz`);
  check(`${name}: >= 97 % of the complete packets decoded`, got >= 0.97 * exp, `${got}/${exp}`);
  check(`${name}: no false CRC-ok packets`, fp === 0, String(fp));
  check(`${name}: CFO estimate within 15 kHz`, cfoErr / Math.max(1, got) < 15e3, (cfoErr / Math.max(1, got) / 1e3).toFixed(1) + ' kHz');
  if (bleDecode) check(`${name}: at least as many as the app's float snapshot decoder`, got >= ref - 2, `${got} vs ${ref}`);
}
// ---------------- 2. sensitivity: BLE at low amplitude ----------------
{
  const r = rng(5), rows = [];
  for (const amp of [0.1, 0.07, 0.05, 0.04, 0.03]) {
    const pk = blePackets(r, 120, { ampLo: amp, ampHi: amp });
    const g = genStream({ fs: 8e6, seconds: 0.2, packets: pk, seed: 11 });
    const res = runGfsk('ble_sens', g, { R: 2, sps: 4, mode: 0, len: 37, sync: '8E89BED6' });
    let exp = 0, got = 0;
    for (const t of g.truth) { if (!res.inside(t)) continue; exp++; if (res.pk.some(p => (p.flags & 1) && Buffer.compare(Buffer.from(p.data.subarray(0, t.pdu.length)), Buffer.from(t.pdu)) === 0)) got++; }
    let ref = 0; const bl = res.words + res.gap;
    if (bleDecode) for (let b = 0; b * bl + res.words <= g.N; b++) ref += bleDecode(g.iq.subarray(2 * b * bl, 2 * (b * bl + res.words)), 8e6, 37).length;
    const snr = 10 * Math.log10(amp * amp / (2 * 0.02 * 0.02));
    rows.push(`amp ${amp} (SNR ${snr.toFixed(1)} dB in 8 MHz): fw ${got}/${exp}, float ref ${bleDecode ? ref : "n/a"}`);
    if (amp === 0.07) check('BLE at SNR 7.9 dB in 8 MHz (~14 dB in 2 MHz): >= 90 % decoded', got >= 0.9 * exp, `${got}/${exp}`);
    if (bleDecode) check(`BLE amp ${amp}: firmware >= app float decoder`, got >= ref, `${got} vs ${ref}`);
  }
  rows.forEach(x => console.log('  sens ' + x));
}
// ---------------- 3. nRF24/ESB-style 2 Mbit GFSK, sync word 0xE7E7E7E7 (MSB first), 8 MS/s R=1 sps 4 ----------------
{
  const r = rng(21), pk = [];
  for (let k = 0; k < 200; k++) {
    const pay = Array.from({ length: 12 }, () => Math.floor(r() * 256));
    const bits = [...msb(0xaa, 8), ...msb(0xe7e7e7e7, 32), ...msb(0xe7, 8), ...pay.flatMap(x => msb(x, 8)), ...msb(0x1234, 16)];
    pk.push({ bits, rate: 2e6, dev: 320e3, cfo: (2 * r() - 1) * 80e3, amp: 0.1 + 0.3 * r(), pay: Uint8Array.from([0xe7, ...pay]) });
  }
  const g = genStream({ fs: 8e6, seconds: 0.25, packets: pk, seed: 22 });
  for (const [R, sps] of [[1, 4], [2, 2]]) {
    const res = runGfsk(`nrf2m_r${R}`, g, { R, sps, mode: 1, len: 13, sync: 'E7E7E7E7', nbits: 32, maxerr: 1 });
    let exp = 0, got = 0;
    for (const t of g.truth) { if (!res.inside(t)) continue; exp++; if (res.pk.some(p => Buffer.compare(Buffer.from(p.data), Buffer.from(t.pay)) === 0)) got++; }
    console.log(`  nrf2m R=${R} sps=${sps}: ${res.stat}; ${got}/${exp} exact payloads`);
    check(`nRF-style 2 Mbit GFSK (R=${R}, ${sps} sps): >= 95 % exact payloads`, got >= 0.95 * exp, `${got}/${exp}`);
  }
}
// ---------------- 4. CC2500-style 250 kbit 2-FSK (no Gaussian), sync 0xD391D391, 8 MS/s R=4 sps 8 ----------------
{
  const r = rng(31), pk = [];
  for (let k = 0; k < 150; k++) {
    const pay = Array.from({ length: 16 }, () => Math.floor(r() * 256));
    const bits = [...msb(0xaaaaaaaa, 32), ...msb(0xd391d391, 32), ...pay.flatMap(x => msb(x, 8))];
    pk.push({ bits, rate: 250e3, dev: 47e3, cfo: (2 * r() - 1) * 30e3, amp: 0.1 + 0.3 * r(), pay: Uint8Array.from(pay), gapUs: 300 + 2000 * r() });
  }
  const g = genStream({ fs: 8e6, seconds: 0.9, packets: pk, bt: 0, seed: 32 });
  const res = runGfsk('cc250k', g, { R: 4, sps: 8, mode: 1, len: 16, sync: 'D391D391', nbits: 32, maxerr: 1 });
  let exp = 0, got = 0;
  for (const t of g.truth) { if (!res.inside(t)) continue; exp++; if (res.pk.some(p => Buffer.compare(Buffer.from(p.data), Buffer.from(t.pay)) === 0)) got++; }
  console.log(`  cc250k: ${res.stat}; ${got}/${exp} exact payloads`);
  check('CC2500-style 250 kbit 2-FSK (R=4, 8 sps): >= 95 % exact payloads', got >= 0.95 * exp, `${got}/${exp}`);
}
// ---------------- 5. preamble-only trigger (mode 2), 1 Mbit, unknown address ----------------
{
  const r = rng(41), pk = [];
  for (let k = 0; k < 150; k++) {
    const pay = Array.from({ length: 10 }, () => Math.floor(r() * 256)), pre = r() < 0.5 ? 0xaa : 0x55;
    const bits = [...msb(pre, 8), ...msb(pre, 8), ...pay.flatMap(x => msb(x, 8))];
    pk.push({ bits, rate: 1e6, dev: 160e3, cfo: (2 * r() - 1) * 50e3, amp: 0.15 + 0.25 * r(), pay: Uint8Array.from(pay) });
  }
  const g = genStream({ fs: 8e6, seconds: 0.3, packets: pk, seed: 42 });
  const res = runGfsk('pre1m', g, { R: 2, sps: 4, mode: 2, len: 10, nbits: 14, maxerr: 0 });
  let exp = 0, got = 0;
  for (const t of g.truth) {
    if (!res.inside(t)) continue; exp++;
    // the trigger fires somewhere inside the 16-bit preamble: the payload must appear in the output at a bit offset
    const want = Buffer.from(t.pay.subarray(0, 6)).toString('hex');
    if (res.pk.some(p => { let bits = ''; for (const x of p.data) bits += x.toString(2).padStart(8, '0'); for (let s = 0; s <= 8; s++) { let h = ''; for (let j = 0; j < 6; j++) h += parseInt(bits.substr(s + 8 * j, 8), 2).toString(16).padStart(2, '0'); if (h === want) return true; } return false; })) got++;
  }
  const noise = runGfsk('pre_noise', genStream({ fs: 8e6, seconds: 0.2, packets: [], seed: 43 }), { R: 2, sps: 4, mode: 2, len: 10, nbits: 14, maxerr: 0 });
  console.log(`  pre1m: ${res.stat}; ${got}/${exp} payloads found; noise-only run: ${noise.pk.length} triggers in 0.2 s`);
  check('preamble trigger: >= 95 % of the payloads recovered', got >= 0.95 * exp, `${got}/${exp}`);
  check('preamble trigger on noise: < 500 triggers/s', noise.pk.length / 0.2 < 500, `${noise.pk.length / 0.2}/s`);
}
// ---------------- 6. noise only, BLE mode: no CRC-ok packets ----------------
{
  const g = genStream({ fs: 8e6, seconds: 0.5, packets: [], seed: 51, noise: 0.05 });
  const res = runGfsk('ble_noise', g, { R: 2, sps: 4, mode: 0, len: 37, sync: '8E89BED6' });
  check('BLE on noise: no CRC-ok packets', !res.pk.some(p => p.flags & 1), `${res.pk.length} records, ${res.stat}`);
}
console.log(failures ? `${failures} FAILED` : 'all GFSK tests passed');
process.exit(failures ? 1 : 0);
