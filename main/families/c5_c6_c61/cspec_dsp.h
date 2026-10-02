/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * CSPEC inner DSP (ESPsoup, included by c5_cont.h; also compiles on the host
 * for tests: it only needs <stdint.h>, sin_tab[]/dsp_init_tables() and IRAM_ATTR).
 *
 * One segment = nfft raw dump words -> Hann window -> FFT -> |X|^2 into the
 * line accumulators.  Mixed radix-8/4 decimation in frequency (natural-order
 * input, digit-reversed output):
 *   16 = 4*4, 32 = 8*4, 64 = 8*8, 128 = 8*4*4, 256 = 8*4*8
 *  - unpacking of the 10-bit I/Q words and the window are fused into the first
 *    stage (no separate window/bit-reverse pass); its window values and
 *    twiddles sit in one per-k table, offsets are compile-time constants
 *    (one specialised copy per nfft),
 *  - the last stage (L = 8 or 4) has only trivial twiddles (W8 = (1-j)/sqrt2
 *    inside the radix-8 butterfly) and feeds |X|^2 straight into the line
 *    accumulators, which are kept in FFT output order (cs_hpos[] maps host
 *    bins), one copy per (nfft, accumulate mode) so sh is a constant,
 *  - the radix-4 middle stage (128/256 only) runs k-outer, k = 0 without multiplies.
 * Fixed point as in 0.4.0 (X in 10-bit LSB): window w = 16384 - 16383 cos
 * (Q15); the radix-8 first stage keeps (x * w) >> 13 (4x scale) and drops
 * the scale in its Q30 twiddle multiply (mulh), other twiddles Q14 from
 * sin_tab (exact 16384 at angle 0); products stay inside int32 for
 * nfft <= 256, power per component pre-shifted by sh.
 * Cost at nfft 64 (CSTAT fft, 8192-word blocks): 46 CPU cycles per sample
 * (mode max) instead of ~115 with the 0.4.0 radix-2 code; closer to a
 * double-precision FFT as well (host test dsp/cstest.c). */
#pragma once
#include <stdint.h>

#define CS_NMAX 256u
#define CS_FOLD 7u        /* u32 partial sums: 7 x |X|^2 (<= 2^29 + eps) < 2^32 */
#define CS_W4K 10u        /* radix-4 first stage per k: 4 window values + 3 (c,s) Q14 */
#ifndef CS_ATTR
#define CS_ATTR IRAM_ATTR
#endif
/* radix-8 first stage per k: window at k + p*n/8 and W_n^{mk} (m = 1..7) as Q30
 * for mulh (the windowed samples run at 4x scale through this stage) */
typedef struct { int16_t w[8]; int32_t cs[14]; } cs_k8_t;
static cs_k8_t cs_k8[32];
static int16_t cs_tab[CS_W4K * 4 + 6 * 8];       /* radix-4 first stage (n = 16) or middle-stage twiddles */
static uint16_t cs_hpos[CS_NMAX];                /* host bin -> FFT output position */
static unsigned cs_n;
static int16_t cs_lt[65];                        /* 30103 * log2(1 + j/64) */
static uint8_t cs_lg8[256];                      /* floor(log2(j)) */

static void cs_twid(int16_t *t, unsigned L, unsigned m, unsigned k) {
    unsigned i = (m * k * (1024 / L)) & 1023;
    t[0] = i ? sin_tab[(i + 256) & 1023] : 16384; t[1] = (int16_t)-sin_tab[i];
}
static void cs_init(unsigned n) {
    dsp_init_tables();                           /* sin_tab: window + Q14 twiddles */
    if (cs_n == n) return;
    for (unsigned j = 0; j <= 64; j++) cs_lt[j] = (int16_t)lroundf(30103.0f * log2f(1 + j / 64.0f));
    for (unsigned j = 1; j < 256; j++) { unsigned e = 0; while ((2u << e) <= j) e++; cs_lg8[j] = (uint8_t)e; }
    unsigned rad[3], ns;
    switch (n) {
    case 16: rad[0] = 4; rad[1] = 4; ns = 2; break;
    case 32: rad[0] = 8; rad[1] = 4; ns = 2; break;
    case 64: rad[0] = 8; rad[1] = 8; ns = 2; break;
    case 128: rad[0] = 8; rad[1] = 4; rad[2] = 4; ns = 3; break;
    default: rad[0] = 8; rad[1] = 4; rad[2] = 8; ns = 3; break;
    }
    /* first stage: per k the window at k + p*q, then W_n^{mk}, m = 1..r-1 */
    unsigned r = rad[0], q = n / r;
    int16_t *t = cs_tab;
    for (unsigned k = 0; k < q; k++) {
        int16_t *wv = r == 8 ? cs_k8[k].w : t;
        for (unsigned p = 0; p < r; p++) { unsigned j = k + p * q; wv[p] = (int16_t)(16384 - sin_tab[((j * 1024 / n) + 256) & 1023]); }
        if (r == 4) { for (unsigned m = 1; m < 4; m++) cs_twid(t + 4 + 2 * (m - 1), n, m, k); t += CS_W4K; continue; }
        for (unsigned m = 1; m < 8; m++) {
            float a = 6.28318530718f * (float)((m * k) % n) / (float)n;
            cs_k8[k].cs[2 * (m - 1)] = (int32_t)lroundf(cosf(a) * 1073741824.0f);
            cs_k8[k].cs[2 * (m - 1) + 1] = (int32_t)lroundf(-sinf(a) * 1073741824.0f);
        }
    }
    if (ns == 3)        /* radix-4 middle stage at L = n/8: W_L^{mk}, m = 1..3 */
        for (unsigned k = 0, L = n / 8; k < L / 4; k++, t += 6)
            for (unsigned m = 1; m <= 3; m++) cs_twid(t + 2 * (m - 1), L, m, k);
    /* position p = m1*(n/r1) + m2*(n/(r1 r2)) + ... holds frequency f = m1 + r1*(m2 + r2*(...)) */
    for (unsigned p = 0; p < n; p++) {
        unsigned rem = p, span = n, f = 0, mul = 1;
        for (unsigned s = 0; s < ns; s++) {
            span /= rad[s];
            unsigned m = rem / span; rem %= span;
            f += m * mul; mul *= rad[s];
        }
        /* device bin f (I+jQ) is host frequency -f (the host negates Q) */
        cs_hpos[(n / 2 + n - f) & (n - 1)] = (uint16_t)p;
    }
    cs_n = n;
}

#define CS_UI(v) (((int32_t)((v) << 22)) >> 22)
#define CS_UQ(v) (((int32_t)((v) << 12)) >> 22)
/* (re,im) *= (c + j s), Q14 */
#define CS_CMUL(r, i, c, s) { int32_t _r = (r * c - i * s) >> 14; i = (r * s + i * c) >> 14; r = _r; }
/* radix-4 DIF core: in a..d, out a = a+b+c+d, b = (a-c) - j(b-d), c = (a+c)-(b+d), d = (a-c) + j(b-d) */
#define CS_R4(ar, ai, br, bi, cr, ci, dr, di) {                                   \
    int32_t t0r = ar + cr, t0i = ai + ci, t1r = ar - cr, t1i = ai - ci;           \
    int32_t t2r = br + dr, t2i = bi + di, t3r = bi - di, t3i = dr - br;           \
    ar = t0r + t2r; ai = t0i + t2i; cr = t0r - t2r; ci = t0i - t2i;               \
    br = t1r + t3r; bi = t1i + t3i; dr = t1r - t3r; di = t1i - t3i; }
/* radix-8 DIF core on r0..r7/i0..i7: afterwards slot p holds DFT output y_m
 * with (p -> m) 0->0, 1->2, 2->4, 3->6, 4->1, 5->3, 6->5, 7->7 */
#define CS_SQ 11585       /* 2^14 / sqrt 2 */
#define CS_R8()  {                                                                \
    int32_t u;                                                                    \
    u = r0 - r4; r0 += r4; r4 = u; u = i0 - i4; i0 += i4; i4 = u;                 \
    u = r1 - r5; r1 += r5; r5 = u; u = i1 - i5; i1 += i5; i5 = u;                 \
    u = r2 - r6; r2 += r6; r6 = i2 - i6; i2 += i6; i6 = -u;          /* *(-j) */  \
    u = r3 - r7; r3 += r7; r7 = u; u = i3 - i7; i3 += i7; i7 = u;                 \
    u = ((r5 + i5) * CS_SQ) >> 14; i5 = ((i5 - r5) * CS_SQ) >> 14; r5 = u;  /* *(1-j)/sqrt2 */  \
    u = ((i7 - r7) * CS_SQ) >> 14; i7 = (-(r7 + i7) * CS_SQ) >> 14; r7 = u; /* *(-1-j)/sqrt2 */ \
    CS_R4(r0, i0, r1, i1, r2, i2, r3, i3)                                         \
    CS_R4(r4, i4, r5, i5, r6, i6, r7, i7) }

/* ---- first stage: unpack + window + butterfly + twiddles, w -> x */
static inline int32_t cs_mulh(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 32); }
static inline __attribute__((always_inline)) void cs_win8(const uint32_t *restrict w, int32_t *restrict x, const unsigned n) {
    const unsigned q = n / 8;
    const cs_k8_t *restrict t = cs_k8;
    for (unsigned k = 0; k < q; k++, t++, w++, x += 2) {
        int32_t r0, i0, r1, i1, r2, i2, r3, i3, r4, i4, r5, i5, r6, i6, r7, i7;
        /* windowed sample at 4x scale: (x * w) >> 13 */
#define CS_LD(p) { uint32_t v = w[p * q]; int32_t wv = t->w[p]; r##p = (CS_UI(v) * wv) >> 13; i##p = (CS_UQ(v) * wv) >> 13; }
        CS_LD(0) CS_LD(1) CS_LD(2) CS_LD(3) CS_LD(4) CS_LD(5) CS_LD(6) CS_LD(7)
#undef CS_LD
        CS_R8()
        /* slot p -> output m: W^{mk} (Q30 mulh removes the 4x), stored at eighth m */
        x[0] = r0 >> 2; x[1] = i0 >> 2;
#define CS_ST(p, m) { int32_t c = t->cs[2 * (m - 1)], s = t->cs[2 * (m - 1) + 1];                                          x[2 * (m * q)] = cs_mulh(r##p, c) - cs_mulh(i##p, s);                                              x[2 * (m * q) + 1] = cs_mulh(r##p, s) + cs_mulh(i##p, c); }
        CS_ST(1, 2) CS_ST(2, 4) CS_ST(3, 6) CS_ST(4, 1) CS_ST(5, 3) CS_ST(6, 5) CS_ST(7, 7)
#undef CS_ST
    }
}
static inline __attribute__((always_inline)) void cs_win4(const uint32_t *restrict w, int32_t *restrict x, const unsigned n) {
    const unsigned q = n / 4;
    const int16_t *restrict t = cs_tab;
    for (unsigned k = 0; k < q; k++, t += CS_W4K, w++, x += 2) {
        uint32_t va = w[0], vb = w[q], vc = w[2 * q], vd = w[3 * q];
        int32_t ar = (CS_UI(va) * t[0]) >> 15, ai = (CS_UQ(va) * t[0]) >> 15;
        int32_t br = (CS_UI(vb) * t[1]) >> 15, bi = (CS_UQ(vb) * t[1]) >> 15;
        int32_t cr = (CS_UI(vc) * t[2]) >> 15, ci = (CS_UQ(vc) * t[2]) >> 15;
        int32_t dr = (CS_UI(vd) * t[3]) >> 15, di = (CS_UQ(vd) * t[3]) >> 15;
        CS_R4(ar, ai, br, bi, cr, ci, dr, di)
        int32_t c1 = t[4], s1 = t[5], c2 = t[6], s2 = t[7], c3 = t[8], s3 = t[9];
        CS_CMUL(br, bi, c1, s1) CS_CMUL(cr, ci, c2, s2) CS_CMUL(dr, di, c3, s3)
        x[0] = ar; x[1] = ai; x[2 * q] = br; x[2 * q + 1] = bi;
        x[4 * q] = cr; x[4 * q + 1] = ci; x[6 * q] = dr; x[6 * q + 1] = di;
    }
}
/* ---- radix-4 middle stage, span L, in place; t = its twiddles (6 per k) */
static inline __attribute__((always_inline)) void cs_mid4(int32_t *restrict x, const unsigned n, const unsigned L, const int16_t *restrict t) {
    const unsigned q2 = L / 2;                        /* quarter in int32 units */
    int32_t *const end = x + 2 * n;
    for (unsigned k = 0; k < L / 4; k++, t += 6) {
        const int32_t c1 = t[0], s1 = t[1], c2 = t[2], s2 = t[3], c3 = t[4], s3 = t[5];
        for (int32_t *p = x + 2 * k; p < end; p += 2 * L) {
            int32_t ar = p[0], ai = p[1], br = p[q2], bi = p[q2 + 1], cr = p[2 * q2], ci = p[2 * q2 + 1], dr = p[3 * q2], di = p[3 * q2 + 1];
            CS_R4(ar, ai, br, bi, cr, ci, dr, di)
            if (k) { CS_CMUL(br, bi, c1, s1) CS_CMUL(cr, ci, c2, s2) CS_CMUL(dr, di, c3, s3) }
            p[0] = ar; p[1] = ai; p[q2] = br; p[q2 + 1] = bi; p[2 * q2] = cr; p[2 * q2 + 1] = ci; p[3 * q2] = dr; p[3 * q2 + 1] = di;
        }
    }
}
/* ---- last stage fused with the power, accumulated by mode (0 mean, 1 max, 2 both) */
#define CS_POW(re, im, j) {                                                        \
    int32_t _a = (re) >> sh, _b = (im) >> sh;                                      \
    uint32_t _p = (uint32_t)(_a * _a) + (uint32_t)(_b * _b);                       \
    if (mode != 1) acc[j] += _p;                                                   \
    if (mode != 0 && _p > mx[j]) mx[j] = _p; }
static inline __attribute__((always_inline)) void cs_last4(const int32_t *restrict x, unsigned n, unsigned sh, uint32_t *restrict acc,
                                                            uint32_t *restrict mx, const int mode) {
    for (unsigned j = 0; j < n; j += 4, x += 8) {
        int32_t ar = x[0], ai = x[1], br = x[2], bi = x[3], cr = x[4], ci = x[5], dr = x[6], di = x[7];
        CS_R4(ar, ai, br, bi, cr, ci, dr, di)
        CS_POW(ar, ai, j) CS_POW(br, bi, j + 1) CS_POW(cr, ci, j + 2) CS_POW(dr, di, j + 3)
    }
}
static inline __attribute__((always_inline)) void cs_last8(const int32_t *restrict x, unsigned n, unsigned sh, uint32_t *restrict acc,
                                                            uint32_t *restrict mx, const int mode) {
    for (unsigned j = 0; j < n; j += 8, x += 16) {
        int32_t r0 = x[0], i0 = x[1], r1 = x[2], i1 = x[3], r2 = x[4], i2 = x[5], r3 = x[6], i3 = x[7];
        int32_t r4 = x[8], i4 = x[9], r5 = x[10], i5 = x[11], r6 = x[12], i6 = x[13], r7 = x[14], i7 = x[15];
        CS_R8()
        CS_POW(r0, i0, j) CS_POW(r1, i1, j + 2) CS_POW(r2, i2, j + 4) CS_POW(r3, i3, j + 6)
        CS_POW(r4, i4, j + 1) CS_POW(r5, i5, j + 3) CS_POW(r6, i6, j + 5) CS_POW(r7, i7, j + 7)
    }
}
/* last stage for every (nfft, mode) with sh = log2(nfft) - 6 (>= 0) as a constant */
static void __attribute__((noinline)) CS_ATTR cs_last(const int32_t *restrict x, unsigned n, uint32_t *restrict acc,
                                                         uint32_t *restrict mx, unsigned mode) {
#define CS_L(N, F, SH) case N: if (mode == 0) F(x, N, SH, acc, mx, 0); else if (mode == 1) F(x, N, SH, acc, mx, 1); else F(x, N, SH, acc, mx, 2); break;
    switch (n) { CS_L(16, cs_last4, 0) CS_L(32, cs_last4, 0) CS_L(64, cs_last8, 0) CS_L(128, cs_last4, 1) default: CS_L(256, cs_last8, 2) }
#undef CS_L
}
/* first (+ middle) stages for one nfft, w -> x */
static void __attribute__((noinline)) CS_ATTR cs_front(const uint32_t *restrict w, int32_t *restrict x, unsigned n) {
    switch (n) {
    case 16: cs_win4(w, x, 16); break;
    case 32: cs_win8(w, x, 32); break;
    case 64: cs_win8(w, x, 64); break;
    case 128: cs_win8(w, x, 128); cs_mid4(x, 128, 16, cs_tab); break;
    default: cs_win8(w, x, 256); cs_mid4(x, 256, 32, cs_tab); break;
    }
}
/* One segment: w[0..n) raw words -> |X|^2 added to acc (u32 partial sums,
 * mode 0/2) and max-held in mx (mode 1/2), both in FFT output order. */
static void CS_ATTR cs_segment(const uint32_t *restrict w, int32_t *restrict x, uint32_t *restrict acc, uint32_t *restrict mx,
                                 unsigned n, unsigned sh, unsigned mode) {
    (void)sh;                               /* = log2(n) - 6 for n > 64, built into cs_last */
    cs_front(w, x, n);
    cs_last(x, n, acc, mx, mode);
}
/* u8 = 2*10*log10(p) (0.5 dB) + 12*sh; identical to the former
 * db100_u64(p)/50 + 12*sh (nested floor division = one division; the
 * division by 5000 is a multiply-high, exact for x < 1.5e6). */
static inline uint8_t cs_u8f(uint32_t p, unsigned sh) {
    if (!p) return 0;
    uint32_t v = p, t, e;                                  /* e = floor(log2 p), branch-free */
    e = (uint32_t)(v > 0xffffu) << 4; v >>= e;
    t = (uint32_t)(v > 0xffu) << 3; v >>= t; e |= t;
    e += cs_lg8[v];
    uint32_t frac = (p << (31 - e)) >> 25 & 63;           /* 6 bits below the leading one */
    uint32_t x = e * 30103u + (uint32_t)cs_lt[frac];
    uint32_t r = (uint32_t)(((uint64_t)x * 858994u) >> 32) + 12 * sh;
    return r > 255 ? 255 : (uint8_t)r;
}
