/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESP32-C5 on-device DSP (ESPsoup extension, receive only), included by c5_opt.h:
 *  - DDC8/DDC16: anti-aliased, decimated snapshots (capture at 20/40/80 MS/s,
 *    DC removal, NCO, Kaiser FIR, decimation) instead of the sample dropping
 *    of the 4/8/10 MS/s hardware rates,
 *  - SPEC (three arguments, "SPECDB" in CAPS) and SWEEP: averaged power
 *    spectra in 0.01 dB computed on the device, and a device-side panorama,
 *  - fixed-point building blocks shared with the continuous engine (CIC3,
 *    streaming FIR decimator, sine table).  See docs/espsoup-c5.md. */

/* ------------------------------------------------------ fixed-point DSP */
static int16_t sin_tab[1024];                   /* Q14 sine, cos = sin(x + 256) */
static void dsp_init_tables(void) {
    static bool done;
    if (done) return;
    for (unsigned j = 0; j < 1024; j++) sin_tab[j] = (int16_t)lroundf(16383.0f * sinf(2.0f * (float)M_PI * j / 1024.0f));
    done = true;
}
static float bessel_i0(float x) {
    float s = 1, t = 1;
    for (int k = 1; k < 25; k++) { t *= (x / (2 * k)) * (x / (2 * k)); s += t; }
    return s;
}
/* Kaiser-windowed low-pass, cutoff fc (cycles/sample), odd length, Q15, unity DC gain. */
#define FIR_MAX 1025
static int16_t fir_h[FIR_MAX];
static unsigned fir_len, fir_nnz;
static uint16_t fir_nz[FIR_MAX / 2 + 1];   /* non-zero taps t < centre */
static float fir_key[3];
static void fir_design(unsigned taps, float fc, float beta) {
    if (fir_key[0] == taps && fir_key[1] == fc && fir_key[2] == beta) return;
    /* two passes (sum, then scaled taps) instead of a 4 KB float buffer: static
     * SRAM below the dump bank is scarce, and the design is cached by its key */
    float sum = 0, m = (taps - 1) / 2.0f, i0b = bessel_i0(beta);
    for (unsigned pass = 0; pass < 2; pass++)
        for (unsigned j = 0; j < taps; j++) {
            float x = j - m, s = x == 0 ? 2 * fc : sinf(2 * (float)M_PI * fc * x) / ((float)M_PI * x);
            float r = x / m, win = bessel_i0(beta * sqrtf(fmaxf(0, 1 - r * r))) / i0b;
            if (pass) fir_h[j] = (int16_t)lroundf(s * win / sum * 32767.0f); else sum += s * win;
        }
    fir_len = taps; fir_key[0] = taps; fir_key[1] = fc; fir_key[2] = beta;
    fir_nnz = 0;
    for (unsigned j = 0; j < (taps - 1) / 2; j++) if (fir_h[j]) fir_nz[fir_nnz++] = j;
}

/* CIC3 decimator state (integer, wrap-around arithmetic). */
typedef struct { int32_t a1, a2, a3, b1, b2, b3, da1, da2, da3, db1, db2, db3; unsigned R, k, sh; } cic3_t;
static unsigned __attribute__((noinline)) IRAM_ATTR cic3_run(cic3_t *c, const uint32_t *w, unsigned n, int32_t *out) {
    int32_t a1 = c->a1, a2 = c->a2, a3 = c->a3, b1 = c->b1, b2 = c->b2, b3 = c->b3;
    unsigned k = c->k, R = c->R, o = 0, i = 0;
    while (i < n) {
        unsigned m = R - k; if (m > n - i) m = n - i;
        const uint32_t *q = w + i, *e = q + m;
        /* loads hoisted 4 ahead: the C5 core stalls on load-use */
        while (q + 4 <= e) {
            uint32_t x0 = q[0], x1 = q[1], x2 = q[2], x3 = q[3]; q += 4;
            int32_t I0 = ((int32_t)(x0 << 22)) >> 22, Q0 = ((int32_t)(x0 << 12)) >> 22;
            int32_t I1 = ((int32_t)(x1 << 22)) >> 22, Q1 = ((int32_t)(x1 << 12)) >> 22;
            int32_t I2 = ((int32_t)(x2 << 22)) >> 22, Q2 = ((int32_t)(x2 << 12)) >> 22;
            int32_t I3 = ((int32_t)(x3 << 22)) >> 22, Q3 = ((int32_t)(x3 << 12)) >> 22;
            a1 += I0; a2 += a1; a3 += a2; b1 += Q0; b2 += b1; b3 += b2;
            a1 += I1; a2 += a1; a3 += a2; b1 += Q1; b2 += b1; b3 += b2;
            a1 += I2; a2 += a1; a3 += a2; b1 += Q2; b2 += b1; b3 += b2;
            a1 += I3; a2 += a1; a3 += a2; b1 += Q3; b2 += b1; b3 += b2;
        }
        while (q < e) {
            uint32_t x = *q++;
            int32_t I = ((int32_t)(x << 22)) >> 22, Q = ((int32_t)(x << 12)) >> 22;
            a1 += I; a2 += a1; a3 += a2; b1 += Q; b2 += b1; b3 += b2;
        }
        i += m; k += m;
        if (k == R) {
            k = 0;
            int32_t y1 = a3 - c->da1; c->da1 = a3; int32_t y2 = y1 - c->da2; c->da2 = y1; int32_t y3 = y2 - c->da3; c->da3 = y2;
            int32_t z1 = b3 - c->db1; c->db1 = b3; int32_t z2 = z1 - c->db2; c->db2 = z1; int32_t z3 = z2 - c->db3; c->db3 = z2;
            out[o++] = y3 >> c->sh; out[o++] = z3 >> c->sh;
        }
    }
    c->a1 = a1; c->a2 = a2; c->a3 = a3; c->b1 = b1; c->b2 = b2; c->b3 = b3; c->k = k;
    return o / 2;
}
/* Streaming complex FIR decimator on int32 (<= 15 bit) samples. The FIR is
 * symmetric: pairs are folded and zero taps (every other tap of a half-band
 * design) are skipped via fir_nz[]. */
#define STREAM_FIR_MAX 129
typedef struct { int32_t hi[2 * STREAM_FIR_MAX], hq[2 * STREAM_FIR_MAX]; unsigned pos, R, k; } firdec_t;
static unsigned __attribute__((noinline)) IRAM_ATTR firdec_run(firdec_t *restrict f, const int32_t *restrict x, unsigned n, int32_t *restrict out) {
    const unsigned L = fir_len, c = (L - 1) / 2, nz = fir_nnz, R = f->R;
    unsigned pos = f->pos, k = f->k, o = 0;
    int32_t *restrict hi = f->hi, *restrict hq = f->hq;
    const int16_t *restrict h = fir_h; const uint16_t *restrict zi = fir_nz;
    /* 33-tap half-band (R2 = 2): taps 1,3,...,15 are the only non-zero pairs */
    const bool hb33 = L == 33 && nz == 8 && zi[0] == 1 && zi[7] == 15;
    for (unsigned i = 0; i < n; i++) {
        int32_t xi = x[2 * i], xq = x[2 * i + 1];
        hi[pos] = xi; hi[pos + L] = xi; hq[pos] = xq; hq[pos + L] = xq;
        if (++pos == L) pos = 0;
        if (++k == R) {
            k = 0;
            const int32_t *pi = hi + pos, *pq = hq + pos;
            int32_t si = pi[c] * h[c], sq = pq[c] * h[c];
            if (hb33) {
#define HB(t) { int32_t hh = h[t]; si += (pi[t] + pi[32 - t]) * hh; sq += (pq[t] + pq[32 - t]) * hh; }
                HB(1) HB(3) HB(5) HB(7) HB(9) HB(11) HB(13) HB(15)
#undef HB
                out[o++] = si >> 15; out[o++] = sq >> 15;
                continue;
            }
            for (unsigned j = 0; j < nz; j++) {
                unsigned t = zi[j], u = L - 1 - t; int32_t hh = h[t];
                si += (pi[t] + pi[u]) * hh; sq += (pq[t] + pq[u]) * hh;
            }
            out[o++] = si >> 15; out[o++] = sq >> 15;
        }
    }
    f->pos = pos; f->k = k;
    return o / 2;
}

/* ------------------------------------------------------ snapshot DDC */
/* Stage: raw capture at hardware rate fin (index div), DC removal, NCO shift,
 * Kaiser FIR low-pass and decimation by R in one polyphase pass.  The FIR has
 * 16*R+1 taps (<= 1025) and a clean (alias-free, flat) band of about +-38 % of
 * the output rate. */
static const unsigned hw_rates[6] = {80000000, 40000000, 20000000, 10000000, 8000000, 4000000};
static bool ddc_snapshot(unsigned bits, unsigned n_out, unsigned div, unsigned R, int32_t offset_hz, uint32_t *elapsed, unsigned *got) {
    unsigned taps = 16 * R + 1; if (taps > FIR_MAX) taps = FIR_MAX | 1;
    unsigned need = n_out * R + taps;
    if (need > BIG_MAX) { n_out = (BIG_MAX - taps) / R; need = n_out * R + taps; }
    fir_design(taps, 0.5f / R, 7.0f);
    dsp_init_tables();
    const char *err = fast_capture(need, div, elapsed);
    if (err) { reply(err); return false; }
    uint32_t *w = DUMP_BUF;
    /* pass 1: unpack, mean removal, NCO -> packed int16 pairs in place */
    int64_t si = 0, sq = 0;
    for (unsigned j = 0; j < need; j++) { uint32_t x = w[j]; si += ((int32_t)(x << 22)) >> 22; sq += ((int32_t)(x << 12)) >> 22; }
    int32_t mi = (int32_t)(si * 64 / (int64_t)need), mq = (int32_t)(sq * 64 / (int64_t)need);
    uint32_t ph = 0, dph = (uint32_t)(int32_t)llroundf((float)offset_hz / hw_rates[div] * 4294967296.0f);
    for (unsigned j = 0; j < need; j++) {
        uint32_t x = w[j];
        int32_t I = ((((int32_t)(x << 22)) >> 22) * 64) - mi, Q = ((((int32_t)(x << 12)) >> 22) * 64) - mq;
        if (dph) {
            int32_t c = sin_tab[((ph >> 22) + 256) & 1023], s = sin_tab[ph >> 22];
            int32_t I2 = (I * c - Q * s) >> 14, Q2 = (I * s + Q * c) >> 14;
            I = I2; Q = Q2; ph += dph;
        }
        I = I > 32767 ? 32767 : I < -32768 ? -32768 : I; Q = Q > 32767 ? 32767 : Q < -32768 ? -32768 : Q;
        w[j] = (uint16_t)(int16_t)I | ((uint32_t)(uint16_t)(int16_t)Q << 16);
    }
    /* pass 2: FIR + decimate in place; output j (word/half-word j) never
     * overlaps inputs still needed (they start at word j*R > j). */
    int16_t *in = (int16_t *)w;
    for (unsigned j = 0; j < n_out; j++) {
        const int16_t *p = in + 2 * (j * R);
        unsigned cc = (taps - 1) / 2;
        int32_t acc_i = p[2 * cc] * fir_h[cc], acc_q = p[2 * cc + 1] * fir_h[cc];
        for (unsigned z = 0; z < fir_nnz; z++) {
            unsigned t = fir_nz[z], u = taps - 1 - t; int32_t h = fir_h[t];
            acc_i += (p[2 * t] + p[2 * u]) * h; acc_q += (p[2 * t + 1] + p[2 * u + 1]) * h;
        }
        int32_t oi = acc_i >> 15, oq = acc_q >> 15;
        if (bits == 16) { ((int16_t *)w)[2 * j] = oi; ((int16_t *)w)[2 * j + 1] = oq; }
        else { ((int8_t *)w)[2 * j] = oi >> 8; ((int8_t *)w)[2 * j + 1] = oq >> 8; }
    }
    *got = n_out;
    return true;
}

/* ------------------------------------------------------------- FFT */
/* In-place radix-2 complex FFT on int32 pairs, Q14 twiddles from sin_tab. */
static void IRAM_ATTR fft_i32(int32_t *x, unsigned n) {
    for (unsigned i = 1, j = 0; i < n; i++) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { int32_t t = x[2 * i]; x[2 * i] = x[2 * j]; x[2 * j] = t; t = x[2 * i + 1]; x[2 * i + 1] = x[2 * j + 1]; x[2 * j + 1] = t; }
    }
    for (unsigned len = 2; len <= n; len <<= 1) {
        unsigned step = 1024 / len;
        for (unsigned i = 0; i < n; i += len) {
            for (unsigned k = 0; k < len / 2; k++) {
                int32_t c = sin_tab[(k * step + 256) & 1023], s = -sin_tab[(k * step) & 1023];
                int32_t *a = x + 2 * (i + k), *b = x + 2 * (i + k + len / 2);
                int32_t br = (int32_t)(((int64_t)b[0] * c - (int64_t)b[1] * s) >> 14);
                int32_t bi = (int32_t)(((int64_t)b[0] * s + (int64_t)b[1] * c) >> 14);
                b[0] = a[0] - br; b[1] = a[1] - bi; a[0] += br; a[1] += bi;
            }
        }
    }
}
/* 100*log10(v) for v > 0 using log2 via clz and a 64-entry mantissa table. */
static int32_t db100_u64(uint64_t v) {
    if (!v) return -10000;
    static int16_t lt[65]; static bool init;
    if (!init) { for (int j = 0; j <= 64; j++) lt[j] = (int16_t)lroundf(30103.0f * log2f(1 + j / 64.0f)); init = true; }
    int e = 63 - __builtin_clzll(v);
    unsigned frac = e >= 6 ? (unsigned)((v >> (e - 6)) & 63) : (unsigned)((v << (6 - e)) & 63);
    return (int32_t)((e * 30103 + lt[frac]) / 100);   /* 100 * 10*log10(2^e * m) = 3010.3*log2 */
}
/* Averaged power spectrum of navg*nfft samples at hardware rate div. Output
 * bins in host orientation (the host negates Q), from -fs/2 to +fs/2, as
 * int16 in 0.01 dB (arbitrary reference, identical for every call). */
static const char *spec_compute(unsigned div, unsigned nfft, unsigned navg, int16_t *outbins, uint32_t *elapsed) {
    dsp_init_tables();
    unsigned n = nfft * navg;
    const char *err = fast_capture(n, div, elapsed);
    if (err) return err;
    int32_t *x = (int32_t *)(DUMP_BUF + HALF_WORDS);          /* upper half workspace */
    uint64_t *acc = (uint64_t *)(DUMP_BUF + HALF_WORDS + 2 * nfft);
    for (unsigned j = 0; j < nfft; j++) acc[j] = 0;
    int64_t si = 0, sq = 0;
    for (unsigned j = 0; j < n; j++) { uint32_t w = DUMP_BUF[j]; si += ((int32_t)(w << 22)) >> 22; sq += ((int32_t)(w << 12)) >> 22; }
    int32_t mi = (int32_t)(si * 16 / (int64_t)n), mq = (int32_t)(sq * 16 / (int64_t)n);
    for (unsigned a = 0; a < navg; a++) {
        const uint32_t *w = DUMP_BUF + a * nfft;
        for (unsigned j = 0; j < nfft; j++) {
            int32_t win = 16384 - sin_tab[(((j * 1024) / nfft) + 256) & 1023];   /* Hann, Q14 (0..32767) */
            int32_t I = ((((int32_t)(w[j] << 22)) >> 22) * 16 - mi), Q = ((((int32_t)(w[j] << 12)) >> 22) * 16 - mq);
            x[2 * j] = (I * win) >> 15; x[2 * j + 1] = (Q * win) >> 15;
        }
        fft_i32(x, nfft);
        for (unsigned j = 0; j < nfft; j++) acc[j] += (uint64_t)((int64_t)x[2 * j] * x[2 * j]) + (uint64_t)((int64_t)x[2 * j + 1] * x[2 * j + 1]);
    }
    /* device bin k (I+jQ) is host frequency -k: host bin h (0..nfft-1, -fs/2..)
     * holds device frequency -(h - nfft/2) -> device index (nfft/2 - h) mod nfft */
    for (unsigned h = 0; h < nfft; h++) {
        unsigned k = (nfft / 2 + nfft - h) % nfft;
        outbins[h] = (int16_t)(db100_u64(acc[k] / navg) - 6000);
    }
    return NULL;
}

static int16_t spec_bins[1024];   /* SPEC and SWEEP (one shared buffer: BSS must stay below the dump bank) */

/* ------------------------------------------------------------ commands */
static bool dsp_command(const char *line) {
    unsigned n, rate, bits = 0; char extra; int32_t off = 0;
    if (!strcmp(line, "DSPBENCH")) {
        /* diagnostic: cycles for 2048 samples of CIC3 (R=20) from bank-1 SRAM vs heap, and a plain load loop */
        static cic3_t cb; int32_t outv[2 * 128]; char h[160];
        uint32_t *heap = heap_caps_malloc(2048 * 4, MALLOC_CAP_8BIT);
        if (!heap) { reply("ERR mem\n"); return true; }
        for (unsigned j = 0; j < 2048; j++) heap[j] = DUMP_BUF[j];
        memset(&cb, 0, sizeof(cb)); cb.R = 20;
        uint32_t c0 = esp_cpu_get_cycle_count(); cic3_run(&cb, DUMP_BUF, 2048, outv);
        uint32_t c1 = esp_cpu_get_cycle_count(); cic3_run(&cb, heap, 2048, outv);
        uint32_t c2 = esp_cpu_get_cycle_count(); volatile uint32_t acc = 0; for (unsigned j = 0; j < 2048; j++) acc += DUMP_BUF[j];
        uint32_t c3 = esp_cpu_get_cycle_count(); for (unsigned j = 0; j < 2048; j++) acc += heap[j];
        uint32_t c4 = esp_cpu_get_cycle_count();
        int64_t t0 = esp_timer_get_time(); uint32_t c5 = esp_cpu_get_cycle_count();
        while (esp_timer_get_time() - t0 < 1000) {}
        uint32_t c6 = esp_cpu_get_cycle_count();
        snprintf(h, sizeof(h), "DSPBENCH cic_bank1 %" PRIu32 " cic_heap %" PRIu32 " load_bank1 %" PRIu32 " load_heap %" PRIu32 " cycles_per_ms %" PRIu32 "\n",
                 c1 - c0, c2 - c1, c3 - c2, c4 - c3, c6 - c5);
        free(heap); reply(h); return true;
    }
    /* DDC8/DDC16 <n_out> <out_rate_hz> [<offset_hz> [<src_rate_idx>]]:
     * anti-aliased decimated snapshot.  Header DATAT; payload int8/int16 I,Q. */
    unsigned src = 99; int m;
    if (((m = sscanf(line, "DDC8 %u %u %" SCNd32 " %u %c", &n, &rate, &off, &src, &extra)) >= 2 && m <= 4 && (bits = 8)) ||
        ((m = sscanf(line, "DDC16 %u %u %" SCNd32 " %u %c", &n, &rate, &off, &src, &extra)) >= 2 && m <= 4 && (bits = 16))) {
        if (m < 3) off = 0;
        unsigned div = 99, R = 0;
        if (src <= 5) { div = src; if (rate && hw_rates[div] % rate == 0) R = hw_rates[div] / rate; }
        else {
            /* default source: 20 MS/s for <= 4 MS/s outputs, else 40/80 MS/s */
            static const unsigned pref[3] = {2, 1, 0};
            for (unsigned j = 0; j < 3 && !R; j++) if (rate && hw_rates[pref[j]] % rate == 0 && hw_rates[pref[j]] / rate >= 2) { div = pref[j]; R = hw_rates[div] / rate; }
        }
        if (!R || R < 2 || R > 128 || n < 16 || n > BIG_MAX || off > (int32_t)hw_rates[div] / 2 || off < -(int32_t)hw_rates[div] / 2) {
            reply("ERR ddc_args\n"); return true;
        }
        prepare_rx();
        uint32_t el; unsigned got;
        if (!ddc_snapshot(bits, n, div, R, off, &el, &got)) return true;
        opt_send_frame(DUMP_BUF, got * (bits / 4), got, el, true);
        return true;
    }
    /* SPEC <rate_idx> <nfft> <navg> -> "SPEC nfft crc us t0" + nfft int16 (0.01 dB).
     * Only the three-argument form; the six/seven-argument upstream SPEC
     * (spectrum.c, SPC1 frames) is a different command handled later. */
    unsigned nfft, navg, f0, f1, st;
    if (sscanf(line, "SPEC %u %u %u %c", &rate, &nfft, &navg, &extra) == 3) {
        if (rate > 5 || nfft < 64 || nfft > 1024 || (nfft & (nfft - 1)) || !navg || nfft * navg > HALF_WORDS - 4) { reply("ERR spec_args\n"); return true; }
        prepare_rx();
        int16_t *bins = spec_bins; uint32_t el;
        const char *err = spec_compute(rate, nfft, navg, bins, &el);
        if (err) { reply(err); return true; }
        uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)bins, nfft * 2);
        char h[96]; snprintf(h, sizeof(h), "SPEC %u %08" PRIx32 " %" PRIu32 " %" PRId64 "\n", nfft, crc, el, opt_last_t0);
        (void)(send_bytes(h, strlen(h)) && send_bytes(bins, nfft * 2));
        return true;
    }
    /* SWEEP <f0_MHz> <f1_MHz> <step_MHz> <rate_idx> <nfft> <navg>: one
     * "SWP mhz nfft crc us" frame per step, then "END SWEEP <steps> <us>". */
    if (sscanf(line, "SWEEP %u %u %u %u %u %u %c", &f0, &f1, &st, &rate, &nfft, &navg, &extra) == 6) {
        if (!frequency_valid(f0) || !frequency_valid(f1) || f1 < f0 || !st || rate > 5 || nfft < 64 || nfft > 1024 ||
            (nfft & (nfft - 1)) || !navg || nfft * navg > HALF_WORDS - 4 || (f1 - f0) / st > 2000) { reply("ERR sweep_args\n"); return true; }
        int16_t *bins = spec_bins;
        int64_t t0 = esp_timer_get_time(); unsigned steps = 0;
        for (unsigned f = f0; f <= f1; f += st) {
            frequency_mhz = f; c5_khz_offset = 0; rx_ready = false; prepare_rx();
            uint32_t el; const char *err = spec_compute(rate, nfft, navg, bins, &el);
            if (err) { reply(err); return true; }
            uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)bins, nfft * 2);
            char h[96]; snprintf(h, sizeof(h), "SWP %u %u %08" PRIx32 " %" PRIu32 "\n", f, nfft, crc, el);
            if (!send_bytes(h, strlen(h)) || !send_bytes(bins, nfft * 2)) return true;
            steps++;
            if (burst_serial_stop_requested()) { burst_serial_drain_stop(); break; }
        }
        char h[64]; snprintf(h, sizeof(h), "END SWEEP %u %" PRId64 "\n", steps, esp_timer_get_time() - t0); reply(h);
        return true;
    }
    return false;
}
