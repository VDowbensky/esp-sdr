/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * On-device demodulators for the ESP32-C5 continuous engine (ESPsoup).
 * Included by c5_demod.h and compiled unchanged on the host by test/demod
 * (needs only <stdint.h>/<string.h>): integer arithmetic only, so the host
 * build is bit-exact with the device.  Input: raw dump words (I = bits 0-9,
 * Q = bits 10-19, signed, device orientation = the host negates Q).
 *
 * VFM  - FM video (analog FPV, PAL/NTSC composite): table-arctangent phase
 *        discriminator -> 0.2 us frequency bins -> sync detection, line
 *        flywheel, vertical sync / line numbering -> per line px luminance
 *        bytes over the active picture.  Output: line records (see vf_block).
 * GFSK - 2-FSK/GFSK packets (BLE, nRF24/ESB, ANT, CC2500-style links):
 *        boxcar decimation, quadrature discriminator Im(y[k] conj y[k-1]),
 *        per-phase bit slicer, sync-word / preamble correlation on every
 *        output sample, packet slicing with a threshold from the sync word,
 *        BLE de-whitening + CRC-24.  Output: packet records (see gf_block).
 */
#pragma once
#include <stdint.h>
#include <string.h>
#include "demod_tab.h"
#ifndef DM_ATTR
#define DM_ATTR
#endif
#ifndef DM_CYC
#define DM_CYC() 0u
#endif
#define DM_INLINE static inline __attribute__((always_inline))
#ifndef VFP_ATTR
#define VFP_ATTR          /* device: IRAM for the PARLIO discriminator (40 MS/s hot loop) */
#endif

/* 10*log10(v) in 0.01 dB for v > 0 (same method as SPEC, constant table) */
static int32_t dm_db100(uint64_t v) {
    if (!v) return -10000;
    int e = 63 - __builtin_clzll(v);
    unsigned frac = e >= 6 ? (unsigned)((v >> (e - 6)) & 63) : (unsigned)((v << (6 - e)) & 63);
    return (int32_t)((e * 30103 + dm_lt[frac]) / 100);
}
static unsigned dm_popc(uint32_t x) { unsigned c = 0; while (x) { x &= x - 1; c++; } return c; }

/* =================================================================== VFM */
#define VF_REC 8u                 /* line record header: u16 line, u8 flags, u8 field count, u32 t (1/16 us) */
#define VF_F_SYNC 1u              /* sync pulse measured on this line (else flywheel) */
#define VF_F_FIELD2 2u
#define VF_F_VBI 4u               /* vertical blanking line (valid with VF_F_VLOCK) */
#define VF_F_VLOCK 8u             /* line number valid */
#define VF_F_VSYNC 16u            /* first broad pulse of a field on this line */
#define VF_F_NTSC 32u
#define VF_F_FULL 64u             /* pixels span sync edge .. next sync edge */
#define VF_SEL_NOVBI 1u
#define VF_SEL_FULL 2u
#define VF_BPU 5                  /* bins per microsecond (0.2 us bins) */

typedef struct { int32_t e8; int32_t len; } vf_run_t;  /* leading edge (Q8 bins, block-local); length in bins (< 0: open at block end) */
typedef struct {
    /* configuration */
    unsigned fin, D, px, std_force, pol_force, sel, rate_Bps;   /* pol_force: 0 auto, else device-domain +1/-1 as 1/2 */
    /* derived by vf_init */
    unsigned B, nd, shl;
    /* caller-provided buffers */
    const uint8_t *tab; int16_t *bins; vf_run_t *runs; unsigned maxruns;
    /* state */
    unsigned qs;
    int locked, vlocked, pol, std;                  /* z = pol * v, sync tip = low z; std 1 PAL, 2 NTSC */
    int32_t tip16, blank16, T16;                    /* z levels (Q4), line period (Q16 bins) */
    int64_t blk0, pred, last_meas, line_ctr, last_meas_line;
    unsigned lineno, since_vs, miss_run, vs_bad, vs_off, acq_wait; int prev_seen, prev_broad;
    uint8_t fieldcnt; int have_blk; unsigned prev_n;
    int32_t bucket;
    /* PARLIO source (vf_block_p8): 1 = bytes from parlio_cap.h; carried phase point, absolute byte count of the
     * next block, prediction floor after a relock (no line twice), phase re-init flag */
    unsigned src, pphase, pinit; int64_t sabs, done16;
    /* colour (VFMCOLOR, PARLIO source only; NULL = off): the raw bytes of the current window = the last cx_keepb
     * bytes of the previous block (cx_tail) followed by the current block (cx_cur); cx_abs0 = absolute byte index of
     * window byte 0 (phase reference of the subcarrier oscillator) */
    struct vc_s *vc;
    const uint8_t *cx_cur, *cx_tail, *px_prev; unsigned cx_keepb, cx_n, px_prev_n; int64_t cx_abs0;
    /* statistics */
    uint32_t blocks, lines, l_sync, l_fly, l_out, l_drop_rate, l_drop_cut, l_vbi, vsyncs, relocks, acq;
    uint64_t cyc_disc, cyc_sync, cyc_slice, cyc_color, samples;
} vf_t;

static const int32_t VF_T16[3] = {0, 320 << 16, 20826457};            /* PAL 64 us, NTSC 63.5556 us (Q16 bins) */
static const int32_t VF_ACT0[3] = {0, 3440640, 3080192};              /* active start 10.5 / 9.4 us (Q16 bins) */
static const int32_t VF_ACTL[3] = {0, 260 << 16, 17235968};           /* active length 52 / 52.6 us */

/* returns 0 on bad configuration */
static int vf_init(vf_t *s) {
    if (s->fin != 20000000u && s->fin != 10000000u) return 0;
    s->B = s->fin / 5000000u;
    if (!s->D) s->D = s->B == 4 ? 2 : 1;                 /* 20 MS/s: phase of every 2nd sample (|f| < 5 MHz) */
    if (s->D > s->B || (s->D & (s->D - 1))) return 0;
    s->nd = s->B / s->D;
    s->shl = s->B == 4 ? 0 : 1;
    if (s->px < 16 || s->px > 256) return 0;
    s->locked = s->vlocked = 0; s->have_blk = 0; s->bucket = 4096; s->fieldcnt = 0; s->lineno = 0;
    return 1;
}

/* Phase discriminator: per 0.2 us bin the mean wrapped phase step per
 * sample in 2*pi/1024 units (f = v * fin / 1024, device orientation).  The
 * phase of every used sample comes from a 64x64 arctangent table indexed by
 * the 6-bit fields [qs, qs+5] of I and Q (qs from the block amplitude). */
#define VF_IX(x) ((((x) >> s1) & 63u) | (((x) >> s2) & 0xfc0u))
/* Phase of every D-th sample, wrapped lag-D differences phi[n] - phi[n-D]
 * summed per bin: the sum telescopes, so the noise of a bin depends on its
 * end-point phases only (not on D), D only limits the range to |f| < fs/2D. */
DM_INLINE void vf_disc_t(const uint32_t *restrict w, unsigned nb, const uint8_t *restrict tab, unsigned s1, int16_t *restrict v,
                         const unsigned B, const unsigned D, const unsigned shl) {
    const unsigned s2 = s1 + 4, nd = B / D;
    unsigned p = tab[VF_IX(w[0])];
    for (unsigned b = 0; b < nb; b++, w += B) {
        int32_t acc = 0;
        for (unsigned j = 0; j < nd; j++) { unsigned c = tab[VF_IX(w[j * D])]; acc += (int32_t)((c - p + 128u) & 255u); p = c; }
        v[b] = (int16_t)((acc - 128 * (int32_t)nd) * (1 << shl));
    }
}
static void __attribute__((noinline)) DM_ATTR vf_disc_41(const uint32_t *w, unsigned nb, const uint8_t *t, unsigned s, int16_t *v) { vf_disc_t(w, nb, t, s, v, 4, 1, 0); }
static void __attribute__((noinline)) DM_ATTR vf_disc_42(const uint32_t *w, unsigned nb, const uint8_t *t, unsigned s, int16_t *v) { vf_disc_t(w, nb, t, s, v, 4, 2, 0); }
static void __attribute__((noinline)) DM_ATTR vf_disc_44(const uint32_t *w, unsigned nb, const uint8_t *t, unsigned s, int16_t *v) { vf_disc_t(w, nb, t, s, v, 4, 4, 0); }
static void __attribute__((noinline)) DM_ATTR vf_disc_21(const uint32_t *w, unsigned nb, const uint8_t *t, unsigned s, int16_t *v) { vf_disc_t(w, nb, t, s, v, 2, 1, 1); }
static void __attribute__((noinline)) DM_ATTR vf_disc_22(const uint32_t *w, unsigned nb, const uint8_t *t, unsigned s, int16_t *v) { vf_disc_t(w, nb, t, s, v, 2, 2, 1); }
static void vf_disc(const vf_t *s, const uint32_t *w, unsigned nb, int16_t *v) {
    switch (s->B * 8 + s->D) {
    case 33: vf_disc_41(w, nb, s->tab, s->qs, v); break;
    case 34: vf_disc_42(w, nb, s->tab, s->qs, v); break;
    case 36: vf_disc_44(w, nb, s->tab, s->qs, v); break;
    case 17: vf_disc_21(w, nb, s->tab, s->qs, v); break;
    default: vf_disc_22(w, nb, s->tab, s->qs, v); break;
    }
}
/* quantisation shift: the 6-bit field [qs, qs+5] must hold the constant FM
 * envelope A with margin: mean(|I|+|Q|) ~ 1.27 A <= 34 * 2^qs */
static unsigned vf_qshift(const uint32_t *w, unsigned n) {
    uint32_t sum = 0, cnt = 0;
    for (unsigned j = 0; j < n; j += 61, cnt++) {
        int32_t I = ((int32_t)(w[j] << 22)) >> 22, Q = ((int32_t)(w[j] << 12)) >> 22;
        sum += (uint32_t)(I < 0 ? -I : I) + (uint32_t)(Q < 0 ? -Q : Q);
    }
    uint32_t m = cnt ? sum / cnt : 0; unsigned q = 0;
    while (q < 4 && m > (34u << q)) q++;
    return q;
}

/* Runs of z = pol*v (smoothed [1 2 1]/4) below thr (gaps <= 2 bins merged).
 * e8 = interpolated crossing (Q8 bins, bin b spans [b, b+1), value at its centre). */
static unsigned DM_ATTR vf_find_runs(const int16_t *v, unsigned nb, int pol, int32_t thr, vf_run_t *runs, unsigned maxr) {
    unsigned nr = 0, first = 0, last = 0, gapc = 0; int in = 0, headless = 0; int32_t e8 = 0;
    if (nb < 4) return 0;
    const int32_t t4 = 4 * thr;                                        /* z below is 4x the smoothed value */
    int32_t zp = pol * (v[0] + 2 * v[1] + v[2]);
    if (zp < t4) { in = 1; headless = 1; first = last = 1; }
    for (unsigned b = 2; b + 1 < nb; b++) {
        int32_t z = pol * (v[b - 1] + 2 * v[b] + v[b + 1]);
        if (z < t4) {
            if (!in) {
                int32_t num = zp - t4, den = zp - z;
                e8 = ((int32_t)b << 8) - 128 + (den > 0 ? (num << 8) / den : 0);
                first = b; in = 1; headless = 0;
            }
            last = b; gapc = 0;
        } else if (in && ++gapc > 2) {
            if (!headless && nr < maxr) { runs[nr].e8 = e8; runs[nr].len = (int32_t)(last - first + 1); nr++; }
            in = 0;
        }
        zp = z;
    }
    if (in && !headless && nr < maxr) { runs[nr].e8 = e8; runs[nr].len = -(int32_t)(last - first + 1); nr++; }
    return nr;
}

/* Locked mode: first run (same rules as vf_find_runs, z scaled by 4) whose start edge
 * lies in bins [b0, b1] and whose length is >= minlen (or open at the block end);
 * the length is measured by walking on.  Cost ~ window + run length instead of nb. */
static int vf_run_in(const int16_t *v, unsigned nb, int pol, int32_t t4, int32_t b0, int32_t b1, int32_t minlen, int32_t *e8, int32_t *len) {
    if (b0 < 2) b0 = 2;
    if (b1 > (int32_t)nb - 2) b1 = (int32_t)nb - 2;
    int32_t zp = b0 <= b1 ? pol * (v[b0 - 2] + 2 * v[b0 - 1] + v[b0]) : 0;
    for (int32_t b = b0; b <= b1; b++) {
        int32_t z = pol * (v[b - 1] + 2 * v[b] + v[b + 1]);
        if (z < t4 && zp >= t4) {
            int32_t den = zp - z, e = (b << 8) - 128 + (den > 0 ? ((zp - t4) << 8) / den : 0);
            int32_t last = b, gapc = 0, q = b + 1;
            for (; q + 1 < (int32_t)nb; q++) {
                int32_t zq = pol * (v[q - 1] + 2 * v[q] + v[q + 1]);
                if (zq < t4) { last = q; gapc = 0; } else if (++gapc > 2) break;
            }
            int32_t L = last - b + 1;
            if (q + 1 >= (int32_t)nb) L = -L;                         /* open at the block end */
            if (L >= minlen || L < 0) { *e8 = e; *len = L; return 1; }
            b = last; z = t4;                                         /* skip this short run */
        }
        zp = z;
    }
    return 0;
}

/* Broad (vertical sync) pulse starting at bin b: >= 70 % of the bins 2..25 us later at sync
 * level (sampled every 3rd bin; a normal or equalising pulse covers < 25 %).  -1: not in block. */
static int vf_broad(const int16_t *v, unsigned nb, int pol, int32_t thr, int32_t b) {
    if (b + 10 < 0 || b + 125 >= (int32_t)nb) return -1;
    unsigned n = 0, k = 0;
    for (int32_t q = b + 10; q < b + 125; q += 3, k++) n += pol * v[q] < thr;
    return n * 10 >= k * 7;
}

/* Acquisition on one block: for each polarity a threshold half-way between
 * the 3 % (sync tip) and 12 % (blanking) quantiles, then H-sync-like runs
 * (3-7 us) spaced by one PAL/NTSC line.  Returns the first good edge (Q16,
 * local) or -1. */
static int64_t vf_acquire(vf_t *s, unsigned nb) {
    uint16_t h[128]; memset(h, 0, sizeof(h));
    const int16_t *v = s->bins;
    for (unsigned b = 0; b < nb; b++) h[(v[b] + 512) >> 3]++;
    /* Both polarities may show periodic 3.4-6 us runs (sync tips, or e.g. a white bar); among
     * the candidates with enough line-spaced pairs take the one whose tip lies furthest from the
     * median (sync tip -0.3 vs. picture mean, white only +0.7 - mean). */
    unsigned need = nb / 320 / 3; if (need < 3) need = 3;
    int bpol = 0, bstd = 1, bgood = 0; int32_t btip = 0, bblank = 0, bext = -1; int64_t bfirst = -1;
    for (int pass = 0; pass < 2; pass++) {
        int pol = pass ? -1 : 1;
        if (s->pol_force && pol != (s->pol_force == 1 ? 1 : -1)) continue;
        unsigned c3 = nb * 3 / 100, c12 = nb * 12 / 100, c50 = nb / 2, acc = 0; int32_t q3 = 0, q12 = 0, q50 = 0; int g3 = 0, g12 = 0;
        for (int k = 0; k < 128; k++) {
            int bk = pol > 0 ? k : 127 - k;
            acc += h[bk];
            int32_t zc = pol * ((bk << 3) - 512 + 4);
            if (!g3 && acc > c3) { q3 = zc; g3 = 1; }
            if (!g12 && acc > c12) { q12 = zc; g12 = 1; }
            if (acc > c50) { q50 = zc; break; }
        }
        if (q12 - q3 < 16) continue;
        int32_t thr = (q3 + q12) / 2;
        unsigned nr0 = vf_find_runs(v, nb, pol, thr, s->runs, s->maxruns), nr = 0;
        /* keep only sync-length (or open) runs: the pair test below needs both ends in that class,
         * and noise produces hundreds of short runs (same result, far fewer pair candidates) */
        for (unsigned j = 0; j < nr0; j++)
            if ((s->runs[j].len >= 17 && s->runs[j].len <= 30) || s->runs[j].len < 0) s->runs[nr++] = s->runs[j];
        int good = 0; int64_t first = -1, sp_sum = 0; int sp_n = 0;
        for (unsigned j = 0; j + 1 < nr; j++) {
            const vf_run_t *a = &s->runs[j];
            if (a->len < 17 || a->len > 30) continue;
            for (unsigned m = j + 1; m < nr && s->runs[m].e8 - a->e8 < 330 * 256; m++) {
                const vf_run_t *c = &s->runs[m];
                int32_t sp = c->e8 - a->e8;
                if ((c->len >= 17 && c->len <= 30) || c->len < 0) {
                    if (sp > 314 * 256 && sp < 323 * 256) {
                        good++; sp_sum += sp; sp_n++;
                        if (first < 0) first = (int64_t)a->e8 << 8;
                    }
                }
            }
        }
        if (good < (int)need) continue;
        int32_t ext = q50 - q3;
        if (ext > bext) {
            bext = ext; bgood = good; bpol = pol; bfirst = first; btip = q3; bblank = q12;
            bstd = sp_n && sp_sum < (int64_t)sp_n * 81637 ? 2 : 1;   /* mean spacing < 318.9 bins -> NTSC */
        }
    }
    if (!bgood) return -1;
    s->pol = bpol; s->std = s->std_force ? (int)s->std_force : bstd;
    s->T16 = VF_T16[s->std]; s->tip16 = btip * 16; s->blank16 = bblank * 16;
    s->locked = 1; s->vlocked = 0; s->miss_run = 0; s->since_vs = 1000; s->last_meas = INT64_MIN; s->acq++;
    s->prev_seen = 0; s->prev_broad = 0; s->vs_bad = 0;
    return bfirst;
}

/* VBI / field of a line number */
static int vf_is_vbi(int std, unsigned ln) {
    if (std == 1) return ln <= 22 || (ln >= 311 && ln <= 335) || ln >= 624;
    return ln <= 21 || (ln >= 264 && ln <= 283);
}
static int vf_field2(int std, unsigned ln) { return std == 1 ? ln >= 313 : ln >= 263; }

/* Slice one line: px pixels between a0 and a0+L (Q16 bins, local), each the
 * exact box average of the instantaneous frequency (prefix sums of the bins,
 * linear inside a bin), mapped to bytes: 0 = sync tip, 64 = blanking,
 * 213 = white (64 + 64 * 0.7/0.3). */
static void VFP_ATTR vf_slice(const vf_t *s, unsigned nb, int64_t a0l, int64_t L, uint8_t *o) {   /* IRAM on the device (hot with colour on) */
    (void)nb; const int16_t *v = s->bins; const unsigned px = s->px;    /* v[nb] = 0 (padding, set by vf_block) */
    const int32_t a0 = (int32_t)a0l, step = (int32_t)L / (int32_t)px;   /* Q16 bins, < 2^29: 32 bit is exact */
    int32_t den = s->blank16 - s->tip16; if (den < 16) den = 16;
    int32_t wq8 = step >> 8; if (wq8 < 1) wq8 = 1;
    /* 32-bit pixel mapping: r = pol*dU - (blanking level over one pixel), clamped so that
     * byte = 64 + K*r (K in Q22) stays inside 0..255 without a second clamp.  No 64-bit
     * division (ROM __divdi3 is slow): a = wq8*den < 2^27, K = floor(2^32 / a) exactly. */
    const uint32_t a = (uint32_t)wq8 * (uint32_t)den;
    const int32_t bq = s->blank16 * wq8 / 16;
    const int32_t K = (int32_t)(0xffffffffu / a + ((0u - a) % a == 0));
    const int32_t lo = -(int32_t)(a / 16), hi = (int32_t)(191 * (a / 1024) + (191 * (a % 1024)) / 1024);
    const int32_t sm = s->pol > 0 ? 0 : -1;                             /* (d ^ sm) - sm = pol * d */
    unsigned jb = (unsigned)(a0 >> 16); int32_t acc = 0;
    int32_t Up = ((a0 >> 8) & 255) * v[jb], x = a0;
    for (unsigned p = 0; p < px; p++) {
        x += step;
        unsigned fl = (unsigned)(x >> 16);
        while (jb < fl) acc += v[jb++];
        int32_t U = acc * 256 + ((x >> 8) & 255) * v[fl];
        int32_t r = (((U - Up) ^ sm) - sm) - bq; Up = U;
        r = r < lo ? lo : r; r = r > hi ? hi : r;
        o[p] = (uint8_t)(64 + ((K * r + (1 << 21)) >> 22));
    }
}
static int32_t vf_mean_z(const vf_t *s, int32_t b0, int32_t b1) {      /* Q4 mean of z over bins [b0, b1) */
    int32_t a = 0; for (int32_t b = b0; b < b1; b++) a += s->bins[b];
    return s->pol * a * 16 / (b1 - b0);
}

/* ============================================== VFM colour (CAPS VFMCOLOR, PARLIO source)
 * The colour subcarrier (PAL 4.43361875 MHz, NTSC 3.579545 MHz) is a frequency modulation of the FM carrier at that
 * rate.  For selected lines the phase steps of the raw bytes (lag `step` bytes, 2*pi/256 units, +128 offset) are
 * mixed with a subcarrier oscillator whose phase is the ABSOLUTE byte index times fsc/40 MHz (one phase-continuous
 * oscillator for the whole run) and summed over the burst (central 70 %) and over nseg segments of the active line.
 * Mixing cheaply: P consecutive samples span almost exactly an integer number of subcarrier cycles (PAL 9 bytes =
 * 0.9976 cycles, NTSC 11 bytes = 0.984), so the steps are first summed into P bins by (sample index mod P) over up to
 * G groups (residual phase slip <= 30 deg, the bins are weighted with the phase at the centre of the run), then the P
 * bins are combined with one sine-table weight each: ~7 operations per byte instead of two multiplies.  Subtracting
 * the bin mean removes the luma (DC) leakage exactly.
 * Reference: the bursts of the colour line and of the line before it (computed only there); their sum points to
 * the mean burst axis (PAL: the bisector of the +-45 deg swing, NTSC: the burst itself), the PAL V switch of the line
 * is the side of its own burst.  U/V are rotated to that axis and scaled to luma steps (149 steps = 0.7 V, from the
 * sync tip / blanking levels of the line logic) - so the chroma saturation does not depend on the burst amplitude,
 * which is reported separately (host: automatic colour control = scale by nominal burst / measured burst).
 * Record (VC_REC + 2 * nseg bytes): u16 line, u8 flags, u8 burst amplitude (luma steps), u32 t16 (= the t16 of the
 * luma record of the same line), u8 burst phase and u8 reference phase used for this line (2*pi/256, against the
 * run's absolute oscillator: a host can smooth the reference over many lines and re-rotate U/V), then nseg x
 * (i8 U, i8 V) in luma steps (U = 0.493 (B'-Y'), V = 0.877 (R'-Y') of the white-normalised picture times 149). */
#define VC_REC 10u
#define VC_F_VNEG 1u              /* PAL: V switch -1 on this line (already applied to V) */
#define VC_F_REF2 2u              /* reference from this and the previous line's burst (else the smoothed reference) */
#define VC_F_WEAK 4u              /* burst below 4 luma steps: no colour (colour killer) */
#define VC_F_VLOCK 8u
#define VC_F_FIELD2 16u
#define VC_F_NTSC 32u
#define VC_TMP 560u
typedef struct vc_s {
    unsigned nseg, every, step;                         /* config: segments per line 4..64, every n-th line, phase of every step-th byte (1, 2, 4) */
    /* derived by vc_setup for the detected standard */
    int std; unsigned P, G, segb, sgr, soff, a0, b0, bgr, boff, gc; uint32_t incb; int16_t wc[11], ws[11];   /* bin weights, Q10 */
    /* state */
    int32_t pbr, pbi; uint32_t pb_lc; int pb_ok;        /* burst of the line before a colour line (>> 4) */
    unsigned rang;                                      /* reference phase of the last two-burst line (2*pi/256), fallback */
    uint8_t *out; unsigned max, n;                      /* records of the current window (caller) */
    uint32_t rate; int32_t bucket;                      /* own USB budget (bytes/s, 0 = unlimited), refilled by vf_block_p8 */
    uint8_t tmp[VC_TMP];                                /* a range that straddles the two window pieces */
    /* statistics */
    uint32_t lines, ref2, weak, drop_rate, drop_full, drop_cut, pre;
} vc_t;

/* returns 0 on bad configuration */
static int vc_init(vc_t *c) {
    if (c->nseg < 4 || c->nseg > 64 || (c->step != 1 && c->step != 2 && c->step != 4) || c->every < 1 || c->every > 625) return 0;
    c->std = 0; c->pb_ok = 0; c->rang = 64; c->n = 0; c->bucket = 4096;
    return 1;
}
static void vc_setup(vc_t *c, int std) {
    const unsigned S = c->step, pal = std != 2;
    c->std = std;
    c->incb = pal ? 476056188u : 384350767u;            /* round(fsc / 40 MHz * 2^32) */
    c->P = pal ? 9 : 11;
    /* the lag-S phase step is a boxcar of S bytes: gain sinc(S fsc / 40 MHz) at the subcarrier, compensated (Q12) */
    static const uint16_t GC[2][3] = {{4180, 4447, 5797}, {4150, 4320, 5106}};
    c->gc = GC[pal ? 0 : 1][S == 1 ? 0 : S == 2 ? 1 : 2];
    int32_t slip = (int32_t)(c->P * S * c->incb);       /* residual phase per group (Q32 cycles) */
    uint32_t as = (uint32_t)(slip < 0 ? -slip : slip);
    c->G = as ? (unsigned)(357913941u / as) : 64;       /* 2^32 / 12: <= 30 deg over one run */
    if (c->G < 1) c->G = 1;
    if (c->G > 64) c->G = 64;
    for (unsigned m = 0; m < c->P; m++) {              /* exp(-j m w_s) of the P bins (Q10), the run phase is applied once per run */
        unsigned i = (unsigned)((m * S * c->incb + (1u << 23)) >> 24) & 255u;
        c->wc[m] = (int16_t)((vc_sin_tab[(i + 64) & 255] + 8) >> 4); c->ws[m] = (int16_t)((vc_sin_tab[i] + 8) >> 4);
    }
    const unsigned AB = pal ? 2080 : 2104;              /* active line 52 / 52.6 us */
    c->a0 = pal ? 420 : 376;                            /* 10.5 / 9.4 us after the sync edge */
    c->segb = AB / c->nseg;
    c->sgr = c->segb / S / c->P; if (!c->sgr) c->sgr = 1;
    c->soff = (c->segb - c->sgr * c->P * S) / 2;
    c->b0 = pal ? 238 : 227;                            /* central 70 % of the burst (5.6 + 2.25 / 5.3 + 2.5 us) */
    const unsigned BL = pal ? 63 : 70;
    c->bgr = BL / S / c->P; if (!c->bgr) c->bgr = 1;
    c->boff = (BL - c->bgr * c->P * S) / 2;
}
/* phase of (re, im) in 2*pi/256 units: scaled to |component| <= 31, then the 6-bit/6-bit cell table of the discriminator */
static unsigned vc_angle(int64_t re, int64_t im) {
    while (re > 31 || re < -31 || im > 31 || im < -31) { re /= 2; im /= 2; }
    while (re && im && re < 16 && re > -16 && im < 16 && im > -16) { re *= 2; im *= 2; }
    return vf_atan_tab[((unsigned)re & 63u) | (((unsigned)im & 63u) << 6)];
}
static uint32_t vc_isqrt64(uint64_t v) {
    uint64_t r = 0, b = 1ull << 62;
    while (b > v) b >>= 2;
    while (b) { if (v >= r + b) { v -= r + b; r = (r >> 1) + b; } else r >>= 1; b >>= 2; }
    return (uint32_t)r;
}
/* P bins of the lag-S phase steps (+128) over g groups of P samples, starting at x (x[-S] must be readable) */
DM_INLINE void vc_acc_t(const uint8_t *restrict x, const uint8_t *restrict tab, unsigned g, int32_t *restrict acc, const unsigned P, const unsigned S) {
    int32_t a[11];
    for (unsigned m = 0; m < P; m++) a[m] = 0;
    unsigned p = tab[x[-(int)S]];
    for (; g; g--, x += P * S) {
        unsigned c[11];
#pragma GCC unroll 11
        for (unsigned m = 0; m < P; m++) c[m] = x[m * S];                  /* loads first (in-order core) */
#pragma GCC unroll 11
        for (unsigned m = 0; m < P; m++) c[m] = tab[c[m]];
#pragma GCC unroll 11
        for (unsigned m = 0; m < P; m++) { a[m] += (int32_t)((c[m] - p + 128u) & 255u); p = c[m]; }
    }
    for (unsigned m = 0; m < P; m++) acc[m] = a[m];
}
static void __attribute__((noinline)) VFP_ATTR vc_acc_9_1(const uint8_t *x, const uint8_t *t, unsigned g, int32_t *a) { vc_acc_t(x, t, g, a, 9, 1); }
static void __attribute__((noinline)) VFP_ATTR vc_acc_9_2(const uint8_t *x, const uint8_t *t, unsigned g, int32_t *a) { vc_acc_t(x, t, g, a, 9, 2); }
static void __attribute__((noinline)) VFP_ATTR vc_acc_11_1(const uint8_t *x, const uint8_t *t, unsigned g, int32_t *a) { vc_acc_t(x, t, g, a, 11, 1); }
static void __attribute__((noinline)) VFP_ATTR vc_acc_11_2(const uint8_t *x, const uint8_t *t, unsigned g, int32_t *a) { vc_acc_t(x, t, g, a, 11, 2); }
static void __attribute__((noinline)) VFP_ATTR vc_acc_9_4(const uint8_t *x, const uint8_t *t, unsigned g, int32_t *a) { vc_acc_t(x, t, g, a, 9, 4); }
static void __attribute__((noinline)) VFP_ATTR vc_acc_11_4(const uint8_t *x, const uint8_t *t, unsigned g, int32_t *a) { vc_acc_t(x, t, g, a, 11, 4); }

/* Window bytes [k - S, k + len) as one pointer (to byte k): directly, or copied into c->tmp if the range straddles
 * the tail of the previous block and the current block.  NULL if outside the window. */
static const uint8_t *vc_ptr(vc_t *c, const vf_t *s, int64_t k, unsigned len) {
    const int64_t S = c->step, kb = s->cx_keepb;
    if (k - S < 0 || k + len > s->cx_n) return NULL;
    if (k - S >= kb) return s->cx_cur + (k - kb);
    if (k + len <= kb) return s->cx_tail + k;
    if (len + S > VC_TMP) return NULL;
    for (int64_t j = k - S; j < k + (int64_t)len; j++) c->tmp[j - (k - S)] = j < kb ? s->cx_tail[j] : s->cx_cur[j - kb];
    return c->tmp + S;
}
/* Mixed sum over gr groups from window byte k: re, im = sum_m (P*a_m - sum a) * exp(-j (theta_m + phoff)) (Q14 scale),
 * runs of <= G groups, theta_m = oscillator phase at sample m of the run's centre group; phoff (Q32 cycles) rotates
 * the result (the reference rotation is folded in here).  0 if outside the window. */
static int VFP_ATTR vc_sum(vc_t *c, const vf_t *s, int64_t k, unsigned gr, uint32_t phoff, int64_t *re, int64_t *im) {
    const unsigned S = c->step, P = c->P, len = gr * P * S;
    const uint8_t *x = vc_ptr(c, s, k, len);
    if (!x) return 0;
    const uint32_t incs = c->incb * S;
    int64_t sr = 0, si = 0;
    while (gr) {
        unsigned g = gr < c->G ? gr : c->G;
        int32_t a[11];
        switch (P * 4 + S) {
        case 37: vc_acc_9_1(x, s->tab, g, a); break;
        case 38: vc_acc_9_2(x, s->tab, g, a); break;
        case 40: vc_acc_9_4(x, s->tab, g, a); break;
        case 45: vc_acc_11_1(x, s->tab, g, a); break;
        case 46: vc_acc_11_2(x, s->tab, g, a); break;
        default: vc_acc_11_4(x, s->tab, g, a); break;
        }
        int32_t sum = 0; for (unsigned m = 0; m < P; m++) sum += a[m];
        /* A - jB = sum_m v_m exp(-j m w_s) with the fixed Q10 bin weights (32 bit: |v| <= 64*255*11, 11 terms) */
        int32_t A = 0, B = 0;
        for (unsigned m = 0; m < P; m++) { int32_t v = a[m] * (int32_t)P - sum; A += v * c->wc[m]; B += v * c->ws[m]; }
        /* run phase: oscillator at the first sample of the centre group (absolute byte k + (g-1)/2 groups), Q14 table */
        uint32_t th = (uint32_t)(s->cx_abs0 + k) * c->incb + (uint32_t)((uint64_t)(g - 1) * P * incs / 2) + phoff + (1u << 23);
        unsigned i = th >> 24; const int32_t cr = vc_sin_tab[(i + 64) & 255], cs = vc_sin_tab[i];
        sr += (int64_t)cr * A - (int64_t)cs * B; si -= (int64_t)cs * A + (int64_t)cr * B;
        x += g * P * S; k += g * P * S; gr -= g;
    }
    *re = (sr + 512) >> 10; *im = (si + 512) >> 10;       /* Q24 -> the Q14 scale of the reference formulas */
    return 1;
}
static int8_t vc_q(int64_t x, int64_t inv, int64_t lim) {   /* round(x * inv / 2^46), clamped to int8 (|x| <= lim first) */
    x = x > lim ? lim : x < -lim ? -lim : x;
    int64_t v = (x * inv + (1ll << 45)) >> 46;
    return (int8_t)(v > 127 ? 127 : v < -127 ? -127 : v);
}
/* Colour of one line (called by vf_lines for every line in order; e = sync edge, Q16 bins of the window) */
static void DM_ATTR vc_line(vf_t *s, int64_t e, unsigned flags, int64_t t0_16) {
    vc_t *c = s->vc;
    const uint32_t lc = (uint32_t)s->line_ctr;
    const int col = lc % c->every == 0 && !(flags & VF_F_VBI), pre = (lc + 1) % c->every == 0;
    if (!col && !pre) return;
    uint32_t c0 = DM_CYC();
    if (c->std != s->std) { vc_setup(c, s->std); c->pb_ok = 0; }
    const unsigned rec = VC_REC + 2 * c->nseg;
    if (col && !pre) {                                   /* nothing to do if the record cannot be sent */
        if (c->n >= c->max) { c->drop_full++; goto out; }
        if (c->rate && c->bucket < (int32_t)rec) { c->drop_rate++; goto out; }
    }
    const int64_t pos = (e * 8) >> 16;                   /* window byte of the sync edge */
    int64_t br, bi;
    if (!vc_sum(c, s, pos + c->b0 + c->boff, c->bgr, 0, &br, &bi)) { c->drop_cut++; c->pb_ok = 0; goto out; }
    const int32_t zr = (int32_t)(br >> 4), zi = (int32_t)(bi >> 4);
    const int had = c->pb_ok && c->pb_lc + 1 == lc; const int32_t ppr = c->pbr, ppi = c->pbi;
    if (pre) { c->pbr = zr; c->pbi = zi; c->pb_lc = lc; c->pb_ok = 1; c->pre++; }
    if (!col || c->n >= c->max || (c->rate && c->bucket < (int32_t)rec)) {
        if (col) { if (c->n >= c->max) c->drop_full++; else c->drop_rate++; }
        goto out;
    }
    {
        int32_t den = s->blank16 - s->tip16; if (den < 64) den = 64;
        unsigned fl = (s->std == 2 ? VC_F_NTSC : 0) | (flags & VF_F_VLOCK ? VC_F_VLOCK : 0) | (flags & VF_F_FIELD2 ? VC_F_FIELD2 : 0);
        /* reference phase (2*pi/256): the sum of this and the previous line's burst, else the last one */
        unsigned ar;
        if (had) { ar = vc_angle((int64_t)zr + ppr, (int64_t)zi + ppi); c->rang = ar; fl |= VC_F_REF2; c->ref2++; }
        else ar = c->rang;
        const unsigned ab = vc_angle(zr, zi);
        /* rotation mapping the reference to +90 deg (rot = j conj(R)/|R|), folded into the oscillator phase of the
         * segment sums: theta' = theta + (ar - 64) * 2^24.  PAL V switch: Re(burst * rot) < 0 <=> burst ahead of R. */
        const uint32_t phoff = (uint32_t)((int32_t)ar - 64) << 24;
        int vsw = 1;
        if (s->std != 2 && (int8_t)(uint8_t)(ab - ar) > 0) { vsw = -1; fl |= VC_F_VNEG; }
#ifdef VC_DEBUG
        VC_DEBUG(lc, pos, zr, zi, ppr, ppi, had, vsw, s->cx_abs0);
#endif
        /* burst amplitude in luma steps: |S| / (P * bytes * den) */
        const int64_t bb = (int64_t)c->P * c->bgr * c->P * c->step * den;
        uint32_t ba = (uint32_t)(((uint64_t)vc_isqrt64((uint64_t)(br >> 4) * (uint64_t)(br >> 4) + (uint64_t)(bi >> 4) * (uint64_t)(bi >> 4)) * 16 * c->gc / 4096 + (uint64_t)bb / 2) / (uint64_t)bb);
        if (ba > 255) ba = 255;
        if (ba < 4) { fl |= VC_F_WEAK; c->weak++; }
        const int64_t D = (int64_t)c->P * c->sgr * c->P * c->step * den, inv = ((1ll << 46) / D) * c->gc / 4096, lim = 128 * D;
        uint8_t *o = c->out + c->n * rec;
        const unsigned ln = (flags & VF_F_VLOCK) ? s->lineno : 0;
        const uint32_t t16 = (uint32_t)(t0_16 + ((e * 209715) >> 32));
        o[0] = (uint8_t)ln; o[1] = (uint8_t)(ln >> 8); o[2] = (uint8_t)fl; o[3] = (uint8_t)ba;
        o[4] = (uint8_t)t16; o[5] = (uint8_t)(t16 >> 8); o[6] = (uint8_t)(t16 >> 16); o[7] = (uint8_t)(t16 >> 24);
        o[8] = (uint8_t)ab; o[9] = (uint8_t)ar;
        int8_t *uv = (int8_t *)(o + VC_REC);
        for (unsigned j = 0; j < c->nseg; j++) {
            int64_t xr, xi;                                  /* S * rot */
            if (!vc_sum(c, s, pos + c->a0 + (int64_t)j * c->segb + c->soff, c->sgr, phoff, &xr, &xi)) { xr = xi = 0; }
            uv[2 * j] = vc_q(-xi, inv, lim);                  /* U = -2 Im / ... */
            uv[2 * j + 1] = vc_q(vsw > 0 ? xr : -xr, inv, lim);
        }
        if (c->rate) c->bucket -= (int32_t)rec;
        c->n++; c->lines++;
    }
out:
    s->cyc_color += DM_CYC() - c0;
}

/* One block of n raw words (n multiple of B): gap_in = input samples missing
 * since the previous block (exact, from the engine), t0_us = time of sample 0.
 * Appends line records (VF_REC + px bytes each) to out; returns the number
 * of lines written. */
static unsigned DM_ATTR vf_lines(vf_t *s, unsigned nb, int64_t t0_16, uint8_t *out, unsigned max_lines, int64_t defer, uint32_t c1);
static unsigned DM_ATTR vf_block(vf_t *s, const uint32_t *w, unsigned n, uint32_t gap_in, int64_t t0_us, uint8_t *out, unsigned max_lines) {
    const unsigned nb = n / s->B;
    uint32_t c0 = DM_CYC();
    if (s->have_blk) s->blk0 += ((int64_t)s->prev_n + gap_in) * 65536 / s->B;
    else { s->blk0 = 0; s->have_blk = 1; }
    s->prev_n = n; s->blocks++; s->samples += n;
    if (s->rate_Bps) {
        s->bucket += (int32_t)(((uint64_t)(n + gap_in) * s->rate_Bps) / s->fin);
        if (s->bucket > 8192) s->bucket = 8192;
    }
    if (!s->locked && s->acq_wait) { s->acq_wait--; return 0; }   /* searching: try every 4th block only (CPU) */
    s->qs = vf_qshift(w, n);
    vf_disc(s, w, nb, s->bins);
    s->bins[nb] = 0;                                     /* padding read by vf_slice at the block end */
    uint32_t c1 = DM_CYC(); s->cyc_disc += c1 - c0;
    return vf_lines(s, nb, t0_us * 16, out, max_lines, 3 << 16, c1);
}

/* Line logic on the bins of one window (nb bins, bin 0 at s->blk0, t0_16 = its time in 1/16 us).  Lines whose
 * edge search would need bins beyond nb16 - defer are left for the next window (engine blocks: lost in the gap;
 * PARLIO windows: the next window starts with the tail of this one).  c1 = cycle stamp after the discriminator. */
static unsigned DM_ATTR vf_lines(vf_t *s, unsigned nb, int64_t t0_16, uint8_t *out, unsigned max_lines, int64_t defer, uint32_t c1) {
    const int64_t nb16 = (int64_t)nb << 16;
    unsigned nl = 0; uint32_t slice_cyc = 0; const uint64_t col0 = s->cyc_color;
    if (!s->locked) {
        int64_t f = vf_acquire(s, nb);
        if (f < 0) { s->acq_wait = 3; s->cyc_sync += DM_CYC() - c1; return 0; }
        s->pred = s->blk0 + f;
        if (s->src) while (s->pred < s->done16) s->pred += s->T16;   /* windows overlap: never a line twice */
    }
    const unsigned nlines = s->std == 1 ? 625 : 525;
    const int64_t W = 3 << 16, T = s->T16;
    const int64_t act0 = (s->sel & VF_SEL_FULL) ? 0 : VF_ACT0[s->std], actL = (s->sel & VF_SEL_FULL) ? T : VF_ACTL[s->std];
    const int32_t t4 = (s->tip16 + s->blank16) / 8;      /* threshold half-way, z scaled by 4 */
    for (;;) {
        int64_t pl = s->pred - s->blk0;
        if (pl + defer >= nb16) break;                   /* edge search needs the next block / window */
        if (pl < -act0) {                                  /* line fell into the gap */
            s->pred += T; s->line_ctr++; s->lines++; s->prev_seen = 0;
            if (s->vlocked) s->lineno = s->lineno % nlines + 1;
            s->since_vs++; continue;
        }
        int32_t re8 = 0, rlen = 0, pb = (int32_t)(pl >> 16);   /* first run >= 1.6 us (or open) starting inside +-W */
        int found = vf_run_in(s->bins, nb, s->pol, t4, pb - 3, pb + 4, 8, &re8, &rlen);
        if (found && (((int64_t)re8 << 8) < pl - W || ((int64_t)re8 << 8) > pl + W)) found = 0;
        /* line reference: flywheel + 1/4 of the measured edge error (PLL; noisy edges at low CNR) */
        int64_t emeas = found ? (int64_t)re8 << 8 : pl, e = pl + (emeas - pl) / 4;
        unsigned flags = s->std == 2 ? VF_F_NTSC : 0;
        if (found) {
            flags |= VF_F_SYNC; s->miss_run = 0; s->l_sync++;
            int64_t ea = s->blk0 + emeas, k = s->line_ctr - s->last_meas_line;
            if (s->last_meas != INT64_MIN && k >= 1 && k <= 8) {
                int64_t per = (int32_t)(ea - s->last_meas) / (int32_t)k, nom = VF_T16[s->std];   /* < 2^28: 32-bit division */
                s->T16 += (int32_t)((per - s->T16) / 16);
                if (s->T16 > nom + nom / 200) s->T16 = (int32_t)(nom + nom / 200);
                if (s->T16 < nom - nom / 200) s->T16 = (int32_t)(nom - nom / 200);
            }
            s->last_meas = ea; s->last_meas_line = s->line_ctr;
            int32_t len = rlen, eb = (int32_t)(e >> 16);
            if (len >= 17 && len <= 30 && eb + 5 >= 0 && eb + 46 <= (int32_t)nb) {   /* H sync: track tip/blank */
                s->tip16 += (vf_mean_z(s, eb + 5, eb + 18) - s->tip16) / 16;
                s->blank16 += (vf_mean_z(s, eb + 40, eb + 46) - s->blank16) / 16;
                if (s->blank16 - s->tip16 < 64) s->blank16 = s->tip16 + 64;
            }
        } else { s->l_fly++; s->miss_run++; }
        /* vertical sync: broad pulses (> 15 us) at the line start (h0) / half a line later (h1).
         * PAL: (E,B) = 313, (B,E) = 3, (B,B) after a seen line without broad pulse = 1.
         * NTSC: (E,B) = 266, (B,E) = 269, (B,B) after such a line = 4.  The first match of a field wins. */
        int h0 = vf_broad(s->bins, nb, s->pol, t4 / 4, (int32_t)(e >> 16));
        int h1 = vf_broad(s->bins, nb, s->pol, t4 / 4, (int32_t)((e + T / 2) >> 16));
        if (s->since_vs > 100 && (h0 > 0 || h1 > 0)) {
            unsigned ln = 0;
            if (h0 == 0 && h1 > 0) ln = s->std == 1 ? 313 : 266;
            else if (h0 > 0 && h1 == 0) ln = s->std == 1 ? 3 : 269;
            else if (h0 > 0 && h1 > 0 && s->prev_seen && !s->prev_broad) ln = s->std == 1 ? 1 : 4;
            /* locked: a vertical sync that contradicts the line count must repeat (same offset) before it is taken */
            if (ln && s->vlocked && ln != s->lineno) {
                unsigned off = (ln + nlines - s->lineno) % nlines;
                if (s->vs_bad && off == s->vs_off) s->vs_bad = 0; else { s->vs_bad = 1; s->vs_off = off; ln = 0; }
            } else if (ln) s->vs_bad = 0;
            if (ln) { s->lineno = ln; s->vlocked = 1; s->since_vs = 0; s->vsyncs++; s->fieldcnt++; flags |= VF_F_VSYNC; }
        }
        s->prev_seen = h0 >= 0 && h1 >= 0; s->prev_broad = h0 > 0 || h1 > 0;
        if (s->vlocked) {
            flags |= VF_F_VLOCK;
            if (vf_field2(s->std, s->lineno)) flags |= VF_F_FIELD2;
            if (vf_is_vbi(s->std, s->lineno)) flags |= VF_F_VBI;
        }
        if (s->sel & VF_SEL_FULL) flags |= VF_F_FULL;
        s->lines++;
        int64_t a0 = e + act0;
        if (a0 < 0 || a0 + actL > nb16) s->l_drop_cut++;
        else if ((s->sel & VF_SEL_NOVBI) && (flags & VF_F_VBI)) s->l_vbi++;
        else if (s->rate_Bps && s->bucket < (int32_t)(VF_REC + s->px)) s->l_drop_rate++;
        else if (nl < max_lines) {
            uint32_t q0 = DM_CYC();
            if (s->rate_Bps) s->bucket -= (int32_t)(VF_REC + s->px);
            uint8_t *o = out + nl * (VF_REC + s->px);
            /* bins are 0.2 us: 1/16 us = 3.2 bins/65536 per Q16 unit -> multiply by round(3.2 * 2^16) / 2^32 */
            uint32_t t16 = (uint32_t)(t0_16 + ((e * 209715) >> 32));
            o[0] = (uint8_t)(s->vlocked ? s->lineno : 0); o[1] = (uint8_t)((s->vlocked ? s->lineno : 0) >> 8);
            o[2] = (uint8_t)flags; o[3] = s->fieldcnt;
            o[4] = (uint8_t)t16; o[5] = (uint8_t)(t16 >> 8); o[6] = (uint8_t)(t16 >> 16); o[7] = (uint8_t)(t16 >> 24);
            vf_slice(s, nb, a0, actL, o + VF_REC);
            nl++; s->l_out++;
            slice_cyc += DM_CYC() - q0;
        } else s->l_drop_rate++;
        if (s->vc && s->src) vc_line(s, e, flags, t0_16);         /* colour (PARLIO windows only, see vf_block_p8) */
        s->pred = s->blk0 + e + T; s->line_ctr++;
        if (s->vlocked) s->lineno = s->lineno % nlines + 1;
        s->since_vs++;
        if (s->miss_run > 48) { s->locked = 0; s->vlocked = 0; s->relocks++; break; }
    }
    uint32_t c2 = DM_CYC();
    s->cyc_slice += slice_cyc; s->cyc_sync += (c2 - c1) - slice_cyc - (uint32_t)(s->cyc_color - col0);
    s->done16 = s->pred;
    return nl;
}

/* ============================================== VFM on the PARLIO source (parlio_cap.h)
 * Input: gap-free bytes from parlio_cap.h at 40 MS/s, low nibble = I, high nibble = Q (a signed 4-bit window of the
 * 10-bit modem samples, device orientation, like the dump words).  The phase of a byte is one lookup in vf_ptab
 * (cell centres, as vf_atan_tab).  Bins of 0.2 us = 8 bytes, phase points at bytes 3 and 7 of each bin (lag 0.1 us:
 * |f| < 5 MHz like 20 MS/s D=2), wrapped differences summed (telescoping): v = phase change over the bin in 2*pi/256
 * units, i.e. the scale of the 20 MS/s path, so the line logic, levels and kHz conversion are shared unchanged
 * (s->fin = 20e6 nominal).  Windows: the last VFP_KEEP bins of the previous window stay in front of the new bins and
 * a line is only cut when the whole line (incl. the vertical-sync probes) lies inside the window (defer = one line),
 * so no line is lost at block boundaries - the stream itself has no gaps. */
#define VFP_KEEP 340u            /* > one line + search window; multiple of 5 -> exact in 1/16 us */
static void __attribute__((noinline)) VFP_ATTR vf_disc_p8(const uint8_t *restrict x, unsigned nb, const uint8_t *restrict tab, unsigned *pp, int16_t *restrict v) {
    unsigned p = *pp, b = 0;
    /* 4 bins per pass, all loads first: the in-order C5 core stalls on load-use (same results as the plain loop) */
    for (; b + 4 <= nb; b += 4, x += 32) {
        unsigned a0 = x[3], a1 = x[7], a2 = x[11], a3 = x[15], a4 = x[19], a5 = x[23], a6 = x[27], a7 = x[31];
        unsigned c0 = tab[a0], c1 = tab[a1], c2 = tab[a2], c3 = tab[a3], c4 = tab[a4], c5 = tab[a5], c6 = tab[a6], c7 = tab[a7];
        v[b] = (int16_t)((int32_t)((c0 - p + 128u) & 255u) + (int32_t)((c1 - c0 + 128u) & 255u) - 256);
        v[b + 1] = (int16_t)((int32_t)((c2 - c1 + 128u) & 255u) + (int32_t)((c3 - c2 + 128u) & 255u) - 256);
        v[b + 2] = (int16_t)((int32_t)((c4 - c3 + 128u) & 255u) + (int32_t)((c5 - c4 + 128u) & 255u) - 256);
        v[b + 3] = (int16_t)((int32_t)((c6 - c5 + 128u) & 255u) + (int32_t)((c7 - c6 + 128u) & 255u) - 256);
        p = c7;
    }
    for (; b < nb; b++, x += 8) {
        unsigned c3 = tab[x[3]], c7 = tab[x[7]];
        v[b] = (int16_t)((int32_t)((c3 - p + 128u) & 255u) + (int32_t)((c7 - c3 + 128u) & 255u) - 256);
        p = c7;
    }
    *pp = p;
}
/* One block of n bytes (n multiple of 8); gap_in = bytes missing before it (ring overrun, else 0); t0_16 = time of
 * x[0] in 1/16 us.  s->bins needs VFP_KEEP + n/8 + 1 entries.  Same record output as vf_block. */
static unsigned DM_ATTR vf_block_p8(vf_t *s, const uint8_t *x, unsigned n, uint32_t gap_in, int64_t t0_16, uint8_t *out, unsigned max_lines) {
    const unsigned nbn = n / 8;
    uint32_t c0 = DM_CYC();
    unsigned keep = 0;
    if (!s->have_blk) { s->have_blk = 1; s->sabs = 0; s->prev_n = 0; s->pinit = 1; s->done16 = 0; }
    s->sabs += gap_in;
    if (gap_in) { s->prev_n = 0; s->pinit = 1; }
    if (s->prev_n) { keep = s->prev_n < VFP_KEEP ? s->prev_n : VFP_KEEP; memmove(s->bins, s->bins + s->prev_n - keep, keep * sizeof(int16_t)); }
    s->blk0 = ((s->sabs >> 3) - (int64_t)keep) << 16;
    s->blocks++; s->samples += n;
    if (s->rate_Bps) {
        s->bucket += (int32_t)(((uint64_t)(n + gap_in) * s->rate_Bps) / 40000000u);
        if (s->bucket > 8192) s->bucket = 8192;
    }
    if (s->vc && s->vc->rate) {
        s->vc->bucket += (int32_t)(((uint64_t)(n + gap_in) * s->vc->rate) / 40000000u);
        if (s->vc->bucket > 8192) s->vc->bucket = 8192;
    }
    if (!s->locked && s->acq_wait) { s->acq_wait--; s->prev_n = 0; s->pinit = 1; s->sabs += n; return 0; }   /* searching: every 4th block */
    if (s->pinit) { s->pphase = s->tab[x[3]]; s->pinit = 0; }
    vf_disc_p8(x, nbn, s->tab, &s->pphase, s->bins + keep);
    const unsigned nb = keep + nbn;
    s->bins[nb] = 0;
    /* colour: raw bytes of this window (the keep bins are the last keep * 8 bytes of the previous block) */
    s->cx_cur = x; s->cx_keepb = keep * 8; s->cx_tail = keep ? s->px_prev + s->px_prev_n - keep * 8 : x;
    s->cx_n = keep * 8 + n; s->cx_abs0 = s->sabs - (int64_t)keep * 8;
    s->prev_n = nb; s->sabs += n;
    uint32_t c1 = DM_CYC(); s->cyc_disc += c1 - c0;
    unsigned nl = vf_lines(s, nb, t0_16 - (int64_t)keep * 16 / 5, out, max_lines, (int64_t)s->T16 + (3 << 16), c1);
    s->px_prev = x; s->px_prev_n = n;
    return nl;
}

/* ================================================================== GFSK */
#define GF_F_CRC 1u         /* BLE: CRC-24 ok */
#define GF_F_TRUNC 2u       /* cut at the block end */
#define GF_F_BLE 4u         /* payload = de-whitened BLE PDU + CRC (LSB-first bytes) */
#define GF_F_LEN 8u         /* BLE: length field above the limit (header only) */
#define GF_MAXBITS 2112u
#define GF_SBUF 560u        /* >= 31*16 + 2*16 + 3 */
typedef struct { uint32_t pos; uint16_t n, off; uint8_t flags, errs; int16_t rssi10; int32_t cfo_hz, dev_hz; } gf_pkt_t;
typedef struct {
    /* configuration */
    unsigned fin, R, sps, mode, nbits, maxerr, len, okonly; int chan;
    uint32_t sync;                                   /* BLE: access address; mode 1: sync word (air order, MSB = first bit) */
    /* derived */
    uint32_t pat, mask, amask;
    uint8_t *bits;                                   /* GF_MAXBITS scratch (caller) */
    int32_t *sbuf;                                   /* GF_SBUF scratch (caller): S of the timing-candidate window */
    /* statistics */
    uint32_t blocks, hits, hit_ovf, pkts, crc_ok, trunc, rejected, pool_ovf;
    uint64_t cyc_scan, cyc_dec, samples;
} gf_t;

static uint32_t gf_rev32(uint32_t x) { uint32_t r = 0; for (int i = 0; i < 32; i++) r |= ((x >> i) & 1u) << (31 - i); return r; }
static int gf_init(gf_t *g) {
    if (g->R != 1 && g->R != 2 && g->R != 4) return 0;
    if (g->sps != 2 && g->sps != 4 && g->sps != 8 && g->sps != 16) return 0;
    if (g->mode > 2 || g->maxerr > 2) return 0;
    if (g->mode == 0) { g->nbits = 32; g->pat = gf_rev32(g->sync); g->mask = 0xffffffffu; g->amask = 0; }
    else if (g->mode == 1) {
        if (g->nbits < 8 || g->nbits > 32) return 0;
        g->mask = g->nbits == 32 ? 0xffffffffu : (1u << g->nbits) - 1; g->pat = g->sync & g->mask; g->amask = 0;
    } else {
        if (g->nbits < 8 || g->nbits > 32) return 0;
        g->mask = (1u << (g->nbits - 1)) - 1; g->pat = g->mask; g->amask = 0xffffffffu;   /* transitions: r ^ (r >> 1) */
    }
    if (g->len < 1 || g->len > 255) return 0;
    return 1;
}
DM_INLINE int32_t dm_I(uint32_t x) { return ((int32_t)(x << 22)) >> 22; }
DM_INLINE int32_t dm_Q(uint32_t x) { return ((int32_t)(x << 12)) >> 22; }

/* Hot loop: y[k] = c[k] + c[k+1] - dc (c = sum of R words; 2 sps: y[k] = c[k+1] - dc), d[k] = Im(y[k] conj y[k-1]),
 * S[k] = sum of the last SPS d's, M = slow mean of S, bit = S > M shifted into one
 * register per sample phase; a hit is a register within 2 bit errors of the pattern. */
DM_INLINE unsigned gf_scan_t(const gf_t *g, const uint32_t *restrict w, unsigned nout, int32_t dci, int32_t dcq, uint32_t *restrict hits, unsigned maxh,
                             uint32_t *ovf, const unsigned R, const unsigned SPS) {
    const uint32_t pat = g->pat, mask = g->mask, am = g->amask;
    uint32_t regs[SPS]; int32_t dpv[SPS];
    for (unsigned j = 0; j < SPS; j++) { regs[j] = 0; dpv[j] = 0; }
    int32_t pi = 0, pq = 0, S = 0, M = 0, ci = 0, cq = 0; unsigned nh = 0;
    for (unsigned j = 0; j < R; j++) { ci += dm_I(w[j]); cq += dm_Q(w[j]); }
    const uint32_t *p = w + R;
    unsigned k = 0;
    for (; k + SPS <= nout;) {
#pragma GCC unroll 16
        for (unsigned ph = 0; ph < SPS; ph++, k++) {
            int32_t ni = 0, nq = 0;
            for (unsigned j = 0; j < R; j++) { uint32_t x = p[j]; ni += dm_I(x); nq += dm_Q(x); }
            p += R;
            int32_t yi, yq;
            if (SPS >= 4) { yi = ci + ni - dci; yq = cq + nq - dcq; ci = ni; cq = nq; }   /* boxcar 2R */
            else { yi = ni - dci; yq = nq - dcq; }                                          /* 2 sps: boxcar R */
            int32_t d = yi * pq - yq * pi; pi = yi; pq = yq;          /* host orientation (device Q negated) */
            S += d - dpv[ph]; dpv[ph] = d;
            M += (S - M) >> 5;
            uint32_t r = (regs[ph] << 1) | (uint32_t)(S > M); regs[ph] = r;
            uint32_t x = ((r ^ ((r >> 1) & am)) ^ pat) & mask;
            x &= x - 1; x &= x - 1;
            if (__builtin_expect(!x, 0)) { if (nh < maxh) hits[nh++] = k; else (*ovf)++; }
        }
    }
    return nh;
}
#define GF_SCAN(R, S) static unsigned __attribute__((noinline)) DM_ATTR gf_scan_##R##_##S(const gf_t *g, const uint32_t *w, unsigned n, int32_t a, int32_t b, uint32_t *h, unsigned m, uint32_t *o) { return gf_scan_t(g, w, n, a, b, h, m, o, R, S); }
GF_SCAN(1, 2) GF_SCAN(1, 4) GF_SCAN(1, 8) GF_SCAN(1, 16)
GF_SCAN(2, 2) GF_SCAN(2, 4) GF_SCAN(2, 8) GF_SCAN(2, 16)
GF_SCAN(4, 2) GF_SCAN(4, 4) GF_SCAN(4, 8) GF_SCAN(4, 16)
static unsigned gf_scan(const gf_t *g, const uint32_t *w, unsigned nout, int32_t dci, int32_t dcq, uint32_t *h, unsigned m, uint32_t *o) {
#define GF_CASE(R, S) case R * 32 + S: return gf_scan_##R##_##S(g, w, nout, dci, dcq, h, m, o);
    switch (g->R * 32 + g->sps) {
    GF_CASE(1, 2) GF_CASE(1, 4) GF_CASE(1, 8) GF_CASE(1, 16)
    GF_CASE(2, 2) GF_CASE(2, 4) GF_CASE(2, 8) GF_CASE(2, 16)
    GF_CASE(4, 2) GF_CASE(4, 4) GF_CASE(4, 8) GF_CASE(4, 16)
    }
#undef GF_CASE
    return 0;
}

/* Slow-path iterator reproducing y/d/S of the hot loop from out-sample k0 on. */
typedef struct { const uint32_t *w; unsigned R, sps, k, nout, ph; int32_t dci, dcq, pi, pq, ci, cq, S; int32_t dpv[16]; uint64_t pow; } gf_it_t;
static void gf_it_start(gf_it_t *it, const uint32_t *w, unsigned R, unsigned sps, unsigned nout, int32_t dci, int32_t dcq, unsigned k0) {
    memset(it, 0, sizeof(*it));
    it->w = w; it->R = R; it->sps = sps; it->nout = nout; it->dci = dci; it->dcq = dcq; it->k = k0; it->ph = k0 % sps;
    for (unsigned j = 0; j < R; j++) { it->ci += dm_I(w[k0 * R + j]); it->cq += dm_Q(w[k0 * R + j]); }
    if (k0) {   /* y[k0-1] for d[k0] */
        int32_t ai = 0, aq = 0;
        for (unsigned j = 0; j < R; j++) { ai += dm_I(w[(k0 - 1) * R + j]); aq += dm_Q(w[(k0 - 1) * R + j]); }
        if (sps >= 4) { it->pi = ai + it->ci - dci; it->pq = aq + it->cq - dcq; }
        else { it->pi = it->ci - dci; it->pq = it->cq - dcq; }
    }
}
/* Advance the iterator (identical arithmetic to the hot loop), specialised per (R, SPS):
 * every stride-th out-sample S is written to out, until nreq values or the block end.
 * Returns the number of values written. */
DM_INLINE unsigned gf_adv_t(gf_it_t *restrict it, int32_t *restrict out, unsigned nreq, unsigned stride, const unsigned R, const unsigned SPS) {
    unsigned k = it->k, ph = it->ph, c = 0, o = 0; const unsigned nout = it->nout;
    int32_t ci = it->ci, cq = it->cq, pi = it->pi, pq = it->pq, S = it->S; const int32_t dci = it->dci, dcq = it->dcq;
    uint64_t pow = it->pow; const uint32_t *p = it->w + (k + 1) * R; int32_t *dpv = it->dpv;
    while (o < nreq && k < nout) {
        int32_t ni = 0, nq = 0;
        for (unsigned j = 0; j < R; j++) { uint32_t x = p[j]; ni += dm_I(x); nq += dm_Q(x); }
        p += R;
        int32_t yi, yq;
        if (SPS >= 4) { yi = ci + ni - dci; yq = cq + nq - dcq; ci = ni; cq = nq; }
        else { yi = ni - dci; yq = nq - dcq; }
        int32_t d = yi * pq - yq * pi; pi = yi; pq = yq;
        S += d - dpv[ph]; dpv[ph] = d; ph = ph + 1 == SPS ? 0 : ph + 1;
        pow += (uint32_t)(yi * yi + yq * yq);                     /* |y| <= 4096: < 2^25 */
        k++;
        if (++c == stride) { out[o++] = S; c = 0; }
    }
    it->k = k; it->ph = ph; it->ci = ci; it->cq = cq; it->pi = pi; it->pq = pq; it->S = S; it->pow = pow;
    return o;
}
#define GF_ADV(R, S) static unsigned __attribute__((noinline)) DM_ATTR gf_adv_##R##_##S(gf_it_t *it, int32_t *o, unsigned n, unsigned st) { return gf_adv_t(it, o, n, st, R, S); }
GF_ADV(1, 2) GF_ADV(1, 4) GF_ADV(1, 8) GF_ADV(1, 16)
GF_ADV(2, 2) GF_ADV(2, 4) GF_ADV(2, 8) GF_ADV(2, 16)
GF_ADV(4, 2) GF_ADV(4, 4) GF_ADV(4, 8) GF_ADV(4, 16)
static unsigned gf_adv(gf_it_t *it, int32_t *o, unsigned n, unsigned st) {
#define GF_ACASE(R, S) case R * 32 + S: return gf_adv_##R##_##S(it, o, n, st);
    switch (it->R * 32 + it->sps) {
    GF_ACASE(1, 2) GF_ACASE(1, 4) GF_ACASE(1, 8) GF_ACASE(1, 16)
    GF_ACASE(2, 2) GF_ACASE(2, 4) GF_ACASE(2, 8) GF_ACASE(2, 16)
    GF_ACASE(4, 2) GF_ACASE(4, 4) GF_ACASE(4, 8) GF_ACASE(4, 16)
    }
#undef GF_ACASE
    return 0;
}

static const uint32_t GF_CRC_INIT = 0x555555u;
/* BLE whitening (x^7 + x^4 + 1, position 0 = 1, 1..6 = channel MSB first) */
static unsigned gf_whiten_init(int ch) {
    unsigned r = 1;
    for (int k = 1; k <= 6; k++) r |= (unsigned)((ch >> (6 - k)) & 1) << k;
    return r;
}

/* Sync-word statistics of one timing candidate: class means of the expected bits, threshold,
 * bit errors and summed eye margin.  Returns 0 if the candidate does not fit. */
static int gf_sync_stats(const gf_t *g, const int32_t *Ssync, int64_t *thr_, int64_t *m1_, int64_t *m0_, uint32_t *exp_, unsigned *errs_, int64_t *eye) {
    const unsigned nb = g->nbits;
    int64_t s1 = 0, s0 = 0; unsigned n1 = 0, n0 = 0;
    int64_t mean = 0; for (unsigned j = 0; j < nb; j++) mean += Ssync[j]; mean /= (int64_t)nb;
    uint32_t got = 0, exp = 0;
    for (unsigned j = 0; j < nb; j++) {
        unsigned e = g->mode == 2 ? (Ssync[j] > mean) : (g->pat >> (nb - 1 - j)) & 1u;
        exp = (exp << 1) | e;
        if (e) { s1 += Ssync[j]; n1++; } else { s0 += Ssync[j]; n0++; }
    }
    if (!n1 || !n0) return 0;
    int64_t m1 = s1 / n1, m0 = s0 / n0;
    if (m1 <= m0) return 0;
    int64_t thr = (m1 + m0) / 2;
    for (unsigned j = 0; j < nb; j++) got = (got << 1) | (uint32_t)(Ssync[j] > thr);
    unsigned errs = g->mode == 2 ? dm_popc(~(got ^ (got >> 1)) & g->mask) : dm_popc((got ^ exp) & (nb == 32 ? 0xffffffffu : (1u << nb) - 1));
    if (errs > g->maxerr) return 0;
    int64_t mg = 0;   /* summed margin of the sync bits on their expected side */
    for (unsigned j = 0; j < nb; j++) { int64_t d = Ssync[j] - thr; mg += ((exp >> (nb - 1 - j)) & 1u) ? d : -d; }
    *eye = mg - (int64_t)errs * (m1 - m0);
    *thr_ = thr; *m1_ = m1; *m0_ = m0; *exp_ = exp; *errs_ = errs;
    return 1;
}

/* Decode the packet whose sync word ends at out-sample k (S integrates its last
 * bit there).  Returns the out-sample of the last decoded bit (0 = rejected). */
static unsigned gf_decode(gf_t *g, const uint32_t *w, unsigned nout, int32_t dci, int32_t dcq, unsigned k, gf_pkt_t *pk, uint8_t *data, unsigned room, int64_t *eye) {
    const unsigned sps = g->sps, nb = g->nbits;
    if (k < (nb - 1) * sps) return 0;
    unsigned kfirst = k - (nb - 1) * sps;                       /* S of the first sync bit */
    unsigned ks = kfirst + 1 > sps ? kfirst + 1 - sps : 0;
    gf_it_t it; gf_it_start(&it, w, g->R, sps, nout, dci, dcq, ks);
    int32_t Ssync[32];
    if (gf_adv(&it, Ssync, 1, kfirst - ks + 1) != 1 || gf_adv(&it, Ssync + 1, nb - 1, sps) != nb - 1) return 0;
    int64_t thr, m1, m0; uint32_t exp; unsigned errs; int64_t ey;
    if (!gf_sync_stats(g, Ssync, &thr, &m1, &m0, &exp, &errs, &ey)) return 0;
    if (eye) { *eye = ey; return 1; }
    /* payload bits (S at the bit ends k + j*sps, in chunks) */
    unsigned want = g->mode == 0 ? 16 : g->len * 8, nbits = 0, flags = 0, kl = k;
    unsigned wr = g->mode == 0 ? gf_whiten_init(g->chan) : 0;
    int32_t chunk[64];
    while (nbits < want) {
        unsigned req = want - nbits; if (req > 64) req = 64;
        if (g->mode == 0 && nbits < 16 && req > 16 - nbits) req = 16 - nbits;
        unsigned got = gf_adv(&it, chunk, req, sps);
        for (unsigned j = 0; j < got; j++) {
            unsigned b = chunk[j] > thr;
#ifdef GF_DEBUG
            GF_DEBUG(nbits, chunk[j], thr, m1, m0, k);
#endif
            if (g->mode == 0) { unsigned o = (wr >> 6) & 1u; wr = ((wr << 1) & 0x7fu) | o; if (o) wr ^= 0x10u; b ^= o; }
            g->bits[nbits++] = (uint8_t)b; kl = k + nbits * sps;
        }
        if (got < req) { flags |= GF_F_TRUNC; break; }
        if (g->mode == 0 && nbits == 16) {
            unsigned L = 0; for (unsigned j = 0; j < 8; j++) L |= (unsigned)g->bits[8 + j] << j;
            if (L > g->len) flags |= GF_F_LEN; else want = 16 + 8 * L + 24;
        }
    }
    unsigned nbytes = (nbits + 7) / 8;
    if (nbytes > room) return 0xffffffffu;                    /* pool full */
    memset(data, 0, nbytes);
    if (g->mode == 0) {
        flags |= GF_F_BLE;
        for (unsigned j = 0; j < nbits; j++) data[j >> 3] |= (uint8_t)(g->bits[j] << (j & 7));
        if (!(flags & (GF_F_TRUNC | GF_F_LEN)) && nbits >= 40) {
            uint32_t c = GF_CRC_INIT; unsigned nb_pdu = nbits - 24, ok = 1;
            for (unsigned j = 0; j < nb_pdu; j++) { unsigned fb = ((c >> 23) & 1u) ^ g->bits[j]; c = (c << 1) & 0xffffffu; if (fb) c ^= 0x65bu; }
            for (unsigned j = 0; j < 24; j++) if (((c >> (23 - j)) & 1u) != g->bits[nb_pdu + j]) { ok = 0; break; }
            if (ok) flags |= GF_F_CRC;
        }
    } else for (unsigned j = 0; j < nbits; j++) data[j >> 3] |= (uint8_t)(g->bits[j] << (7 - (j & 7)));
    /* power over sync + payload, carrier offset and deviation (small-angle, |y|^2 normalised) */
    uint64_t pw = it.pow, cnt = it.k - ks;
    uint64_t P = cnt ? pw / cnt : 0; if (!P) P = 1;
    uint32_t fout = g->fin / g->R; unsigned L = g->sps >= 4 ? 2 * g->R : g->R; uint64_t fs2 = (uint64_t)(L * 512) * (L * 512);
    int64_t q = thr * (int64_t)fout / ((int64_t)sps * (int64_t)P);
    int64_t qd = (m1 - m0) / 2 * (int64_t)fout / ((int64_t)sps * (int64_t)P);
    pk->cfo_hz = (int32_t)(q * 159155 / 1000000);             /* host orientation (d is computed with Q negated) */
    pk->dev_hz = (int32_t)(qd * 159155 / 1000000);
    pk->rssi10 = (int16_t)((dm_db100(P) - dm_db100(fs2)) / 10);
    pk->pos = k + 1 >= nb * sps ? (k + 1 - nb * sps) * g->R : 0;
    pk->n = (uint16_t)nbytes; pk->flags = (uint8_t)flags; pk->errs = (uint8_t)errs;
    return kl;
}

/* One block: returns the number of packet records; payload bytes go to pool
 * (pk[i].off / pk[i].n). */
static unsigned DM_ATTR gf_block(gf_t *g, const uint32_t *w, unsigned n, uint32_t *hits, unsigned maxh, gf_pkt_t *pk, unsigned maxpk, uint8_t *pool, unsigned poolsz) {
    uint32_t c0 = DM_CYC();
    g->blocks++; g->samples += n;
    unsigned nout = n / g->R - 1;
    /* DC of this block (every 8th sample), in y units (2R samples) */
    int32_t si = 0, sq = 0; unsigned cnt = 0;
    for (unsigned j = 0; j < n; j += 8, cnt++) { si += dm_I(w[j]); sq += dm_Q(w[j]); }
    const int32_t L = (int32_t)(g->sps >= 4 ? 2 * g->R : g->R);          /* boxcar length (input samples per y) */
    int32_t dci = (int32_t)((int64_t)si * L / (int32_t)cnt), dcq = (int32_t)((int64_t)sq * L / (int32_t)cnt);
    uint32_t ovf = 0;
    unsigned nh = gf_scan(g, w, nout, dci, dcq, hits, maxh, &ovf);
    g->hit_ovf += ovf; g->hits += nh;
    uint32_t c1 = DM_CYC(); g->cyc_scan += c1 - c0;
    unsigned np = 0, used = 0, busy_until = 0;
    for (unsigned a = 0; a < nh;) {
        unsigned b = a + 1;
        while (b < nh && hits[b] - hits[a] < g->sps) b++;          /* one group = one alignment */
        /* timing: best sync-word eye opening among the group and its neighbours */
        unsigned ka = hits[a] ? hits[a] - 1 : 0, kb = hits[b - 1] + 1, k = hits[a];
        if (kb < hits[a] + g->sps) kb = hits[a] + g->sps;          /* at least one bit period of alignments */
        if (kb >= nout) kb = nout - 1;
        a = b;
        if (k <= busy_until && busy_until) continue;
        int64_t best = INT64_MIN;
        /* S of all out-samples the candidates can need, computed once (identical arithmetic) */
        const unsigned span = (g->nbits - 1) * g->sps, kmax = ka + 2 * g->sps + 2 < nout ? ka + 2 * g->sps + 2 : nout - 1;
        unsigned kA = ka >= span ? ka - span : 0, nS = 0;
        {
            gf_it_t it; unsigned ks = kA + 1 > g->sps ? kA + 1 - g->sps : 0;
            gf_it_start(&it, w, g->R, g->sps, nout, dci, dcq, ks);
            unsigned want = kmax - kA + 1; if (want > GF_SBUF) want = GF_SBUF;
            if (gf_adv(&it, g->sbuf, 1, kA - ks + 1) == 1) nS = 1 + gf_adv(&it, g->sbuf + 1, want - 1, 1);
        }
        for (unsigned c = ka; c <= kb; c++) {
            int64_t e; int okc = 0;
            if (c >= span && c - span >= kA && c - kA < nS) {
                int32_t Ss[32]; unsigned b0 = c - span - kA;
                for (unsigned j = 0; j < g->nbits; j++) Ss[j] = g->sbuf[b0 + j * g->sps];
                int64_t t1, t2, t3; uint32_t t4; unsigned t5;
                okc = gf_sync_stats(g, Ss, &t1, &t2, &t3, &t4, &t5, &e);
            }
#ifdef GF_DEBUG_EYE
            GF_DEBUG_EYE(c, okc, e);
#endif
            if (okc && best == INT64_MIN && kb < c + g->sps && c + g->sps < nout) kb = c + g->sps;   /* a full bit after the first fit */
            if (okc && e > best) { best = e; k = c; }
        }
        if (best == INT64_MIN) { g->rejected++; continue; }
        if (np >= maxpk) { g->pool_ovf++; break; }
        unsigned kl = gf_decode(g, w, nout, dci, dcq, k, &pk[np], pool + used, poolsz - used, NULL);
        if (kl == 0xffffffffu) { g->pool_ovf++; break; }
        if (!kl) { g->rejected++; continue; }
        busy_until = kl;
        g->pkts++;
        if (pk[np].flags & GF_F_TRUNC) g->trunc++;
        if (pk[np].flags & GF_F_CRC) g->crc_ok++;
        if (g->okonly && g->mode == 0 && !(pk[np].flags & GF_F_CRC)) continue;
        pk[np].off = (uint16_t)used; used += pk[np].n; np++;
    }
    g->cyc_dec += DM_CYC() - c1;
    return np;
}
