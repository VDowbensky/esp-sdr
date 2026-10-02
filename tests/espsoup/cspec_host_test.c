/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * Host test of main/families/c5_c6_c61/cspec_dsp.h (CSPEC): the mixed radix-8/4 code against the
 * earlier radix-2 implementation (kept below as reference) and a double-precision DFT.
 * Build: cc -O2 -o cspec_host_test cspec_host_test.c -I../../main/families/c5_c6_c61 -lm
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#define IRAM_ATTR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static int16_t sin_tab[1024];
static void dsp_init_tables(void) { for (unsigned j = 0; j < 1024; j++) sin_tab[j] = (int16_t)lroundf(16383.0f * sinf(2.0f * (float)M_PI * j / 1024.0f)); }
#include "cspec_dsp.h"

/* ---- reference: the earlier radix-2 CSPEC code */
static int32_t db100_u64(uint64_t v) {
    if (!v) return -10000;
    static int16_t lt[65]; static bool init;
    if (!init) { for (int j = 0; j <= 64; j++) lt[j] = (int16_t)lroundf(30103.0f * log2f(1 + j / 64.0f)); init = true; }
    int e = 63 - __builtin_clzll(v);
    unsigned frac = e >= 6 ? (unsigned)((v >> (e - 6)) & 63) : (unsigned)((v << (6 - e)) & 63);
    return (int32_t)((e * 30103 + lt[frac]) / 100);
}
static int16_t o_win[256], o_tw[256]; static uint16_t o_rev[256];
static void o_init(unsigned n) {
    unsigned lg = __builtin_ctz(n);
    for (unsigned j = 0; j < n; j++) {
        o_win[j] = (int16_t)(16384 - sin_tab[((j * 1024 / n) + 256) & 1023]);
        unsigned r = 0; for (unsigned b = 0; b < lg; b++) if (j & (1u << b)) r |= 1u << (lg - 1 - b);
        o_rev[j] = (uint16_t)r;
    }
    for (unsigned k = 0; k < n / 2; k++) { o_tw[2 * k] = sin_tab[((k * 1024 / n) + 256) & 1023]; o_tw[2 * k + 1] = -sin_tab[(k * 1024 / n) & 1023]; }
}
static void o_fft(int32_t *restrict x, unsigned n) {
    for (unsigned i = 0; i < 2 * n; i += 8) {
        int32_t *p = x + i;
        int32_t a0 = p[0] + p[2], a1 = p[1] + p[3], b0 = p[0] - p[2], b1 = p[1] - p[3];
        int32_t c0 = p[4] + p[6], c1 = p[5] + p[7], d0 = p[4] - p[6], d1 = p[5] - p[7];
        p[0] = a0 + c0; p[1] = a1 + c1; p[4] = a0 - c0; p[5] = a1 - c1;
        p[2] = b0 + d1; p[3] = b1 - d0; p[6] = b0 - d1; p[7] = b1 + d0;
    }
    for (unsigned len = 8; len <= n; len <<= 1) {
        unsigned half = len / 2, step = 2 * (n / len);
        for (unsigned i = 0; i < n; i += len) {
            int32_t *a = x + 2 * i, *b = a + 2 * half;
            const int16_t *t = o_tw;
            for (unsigned k = 0; k < half; k++, a += 2, b += 2, t += step) {
                int32_t c = t[0], s = t[1], b0 = b[0], b1 = b[1];
                int32_t br = (b0 * c - b1 * s) >> 14, bi = (b0 * s + b1 * c) >> 14;
                b[0] = a[0] - br; b[1] = a[1] - bi; a[0] += br; a[1] += bi;
            }
        }
    }
}
static inline uint8_t o_u8(uint64_t p, unsigned sh) { int32_t v = db100_u64(p) / 50 + 12 * (int32_t)sh; return !p || v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v; }
static void o_segment(const uint32_t *w, int32_t *x, uint64_t *acc, uint32_t *mx, unsigned nfft, unsigned sh) {
    for (unsigned j = 0; j < nfft; j++) {
        uint32_t v = w[j]; int32_t wn = o_win[j]; unsigned r = o_rev[j];
        x[2 * r] = ((((int32_t)(v << 22)) >> 22) * wn) >> 15;
        x[2 * r + 1] = ((((int32_t)(v << 12)) >> 22) * wn) >> 15;
    }
    o_fft(x, nfft);
    for (unsigned j = 0; j < nfft; j++) {
        int32_t a = x[2 * j] >> sh, b = x[2 * j + 1] >> sh;
        uint32_t p = (uint32_t)(a * a) + (uint32_t)(b * b);
        acc[j] += p; if (p > mx[j]) mx[j] = p;
    }
}

static uint32_t rng = 12345;
static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 8; }
static double gauss(void) { double u = (rnd() + 1.0) / 16777217.0, v = (rnd() + 1.0) / 16777217.0; return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }
static uint32_t pack(int I, int Q) { I = I > 511 ? 511 : I < -512 ? -512 : I; Q = Q > 511 ? 511 : Q < -512 ? -512 : Q; return (uint32_t)(I & 1023) | ((uint32_t)(Q & 1023) << 10) | (rnd() & 0xfff00000u); }

int main(void) {
    dsp_init_tables();
    static uint32_t w[8192]; static int32_t x[512], xo[512];
    static uint64_t acc_o[256], acc_n64[256]; static uint32_t mx_o[256], mx_n[256], acc_n[256];
    int fails = 0;
    cs_init(64);
    for (uint32_t x = 0; x < 1000000; x++) if ((uint32_t)(((uint64_t)x * 858994u) >> 32) != x / 5000) { printf("mulhu mismatch %u\n", x); fails++; break; }
    for (uint64_t p = 1; p < (1ull << 32); p = p * 3 / 2 + 1) for (unsigned sh = 0; sh < 3; sh++) if (cs_u8f((uint32_t)p, sh) != o_u8(p, sh)) { printf("u8 mismatch %llu\n", (unsigned long long)p); fails++; }
    for (uint32_t p = 0; p < 300000; p++) if (cs_u8f(p, 0) != o_u8(p, 0)) { printf("u8 mismatch %u\n", p); fails++; break; }
    for (int sig = 0; sig < 5; sig++) {
        /* 0 noise sigma 8, 1 tone 300 + noise, 2 two tones, 3 full-scale square/clipped, 4 weak noise sigma 1.5 */
        double f1 = 0.1234, f2 = -0.3017;
        for (unsigned j = 0; j < 8192; j++) {
            double I = 0, Q = 0;
            if (sig == 0) { I = 8 * gauss(); Q = 8 * gauss(); }
            if (sig == 1) { I = 300 * cos(2 * M_PI * f1 * j) + 5 * gauss(); Q = 300 * sin(2 * M_PI * f1 * j) + 5 * gauss(); }
            if (sig == 2) { I = 200 * cos(2 * M_PI * f1 * j) + 30 * cos(2 * M_PI * f2 * j) + 3 * gauss(); Q = 200 * sin(2 * M_PI * f1 * j) + 30 * sin(2 * M_PI * f2 * j) + 3 * gauss(); }
            if (sig == 3) { I = (j & 4) ? 511 : -512; Q = -512; }
            if (sig == 4) { I = 1.5 * gauss(); Q = 1.5 * gauss(); }
            w[j] = pack((int)lround(I), (int)lround(Q));
        }
        for (unsigned n = 16; n <= 256; n *= 2) {
            unsigned sh = n > 64 ? __builtin_ctz(n) - 6 : 0, navg = 8, lgn = 3;
            o_init(n); cs_init(n);
            int dmean_max = 0, dmax_max = 0; long dsum = 0, cnt = 0; double ddb_ref = 0, ddb_old = 0, emax_n = 0, emax_o = 0; int ndb = 0;
            for (unsigned line = 0; line < 8192 / (n * navg); line++) {
                memset(acc_o, 0, sizeof(acc_o)); memset(mx_o, 0, sizeof(mx_o)); memset(acc_n64, 0, sizeof(acc_n64)); memset(acc_n, 0, sizeof(acc_n)); memset(mx_n, 0, sizeof(mx_n));
                unsigned fold = 0;
                for (unsigned s = 0; s < navg; s++) {
                    const uint32_t *ws = w + (line * navg + s) * n;
                    o_segment(ws, xo, acc_o, mx_o, n, sh);
                    cs_segment(ws, x, acc_n, mx_n, n, sh, 2);
                    if (++fold == CS_FOLD || s == navg - 1) { for (unsigned j = 0; j < n; j++) { acc_n64[j] += acc_n[j]; acc_n[j] = 0; } fold = 0; }
                }
                for (unsigned hb = 0; hb < n; hb++) {
                    unsigned k = (n / 2 + n - hb) & (n - 1);   /* old: natural order */
                    unsigned p = cs_hpos[hb];
                    int uo = o_u8(acc_o[k] / navg, sh), un = cs_u8f((uint32_t)(acc_n64[p] >> lgn), sh);
                    int mo = o_u8(mx_o[k], sh), mn = cs_u8f(mx_n[p], sh);
                    /* also check cs_u8f == o_u8 on the same value */
                    if (cs_u8f((uint32_t)(acc_o[k] / navg), sh) != uo || cs_u8f(mx_o[k], sh) != mo) { printf("u8 conversion mismatch\n"); fails++; }
                    int d1 = abs(uo - un), d2 = abs(mo - mn);
                    if (d1 > dmean_max) dmean_max = d1;
                    if (d2 > dmax_max) dmax_max = d2;
                    dsum += un - uo; cnt++;
                }
                /* double reference for the mean line: host bin hb -> freq index of conj */
                if (line == 0) {
                    for (unsigned hb = 0; hb < n; hb++) {
                        double pacc = 0;
                        for (unsigned s = 0; s < navg; s++) {
                            const uint32_t *ws = w + s * n; double re = 0, im = 0;
                            int fh = (int)hb - (int)n / 2;     /* host frequency index (host sees I - jQ) */
                            for (unsigned j = 0; j < n; j++) {
                                double I = ((int32_t)(ws[j] << 22)) >> 22, Q = -(double)(((int32_t)(ws[j] << 12)) >> 22);
                                double wv = 0.5 - 0.5 * cos(2 * M_PI * j / n), ph = -2 * M_PI * fh * (double)j / n;
                                re += wv * (I * cos(ph) - Q * sin(ph)); im += wv * (I * sin(ph) + Q * cos(ph));
                            }
                            pacc += (re * re + im * im) / (double)(1u << (2 * sh));
                        }
                        double ref = 20 * log10(pacc / navg) + 12 * sh;
                        int un = cs_u8f((uint32_t)(acc_n64[cs_hpos[hb]] >> lgn), sh);
                        int uo = o_u8(acc_o[(n / 2 + n - hb) & (n - 1)] / navg, sh);
                        if (ref > 30) { ddb_ref += fabs(un + 0.5 - ref); ddb_old += fabs(uo + 0.5 - ref); ndb++;
                            if (fabs(un + 0.5 - ref) > emax_n) emax_n = fabs(un + 0.5 - ref); if (fabs(uo + 0.5 - ref) > emax_o) emax_o = fabs(uo + 0.5 - ref); }
                    }
                }
            }
            printf("sig %d n %3u: new-old |d| max mean %d max %d bias %+.3f | vs double (>15 dB bins): new %.2f/%.1f old %.2f/%.1f (mean/max u8) n=%d\n",
                   sig, n, dmean_max, dmax_max, (double)dsum / cnt, ndb ? ddb_ref / ndb : 0, emax_n, ndb ? ddb_old / ndb : 0, emax_o, ndb);
            if (ndb && ddb_ref > ddb_old * 1.1 + 0.05 * ndb) fails++;
        }
    }
    printf(fails ? "FAIL %d\n" : "PASS\n", fails);
    return fails != 0;
}
