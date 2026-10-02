/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESP32-C5 continuous capture engine (ESPsoup extension, receive only),
 * included by c5_opt.h.  The dump runs back to back in 8192-word blocks; the
 * only gap per block is copying it out of the dump bank (see below), 91-96 %
 * of all samples at 4-10 MS/s.  Users: STREAM8/16 (decimated I/Q stream),
 * CSPEC (continuous spectrogram), CPKT (burst catcher), and the on-device
 * demodulators (c5_demod.h).  Every gap is measured and reported exactly.
 * See docs/espsoup-c5.md for the protocol and measurements. */

/* ------------------------------------------------- USB/UART output ring */
#define STREAM_RING 16384u
static uint8_t *st_ring; static unsigned st_ring_size = STREAM_RING;
static volatile unsigned st_head, st_tail;  /* free-running byte counters */
static bool st_isr;                         /* native USB: ISR drains the ring */
static inline unsigned ring_used(void) { return st_head - st_tail; }
static void ring_put(const void *p, unsigned n) {
    unsigned h = st_head, off = h & (st_ring_size - 1), k = n > st_ring_size - off ? st_ring_size - off : n;
    memcpy(st_ring + off, p, k);
    if (n > k) memcpy(st_ring, (const uint8_t *)p + k, n - k);
    __asm__ volatile("fence rw,rw" ::: "memory");
    st_head = h + n;
}
/* Non-blocking: hand ring bytes to the transport (ISR kick or FIFO polling). */
static void IRAM_ATTR ring_pump(void) {
    if (st_isr) { if (ring_used()) burst_serial_ring_kick(); return; }
    while (ring_used()) {
        unsigned t = st_tail & (st_ring_size - 1), k = ring_used();
        if (k > st_ring_size - t) k = st_ring_size - t;
        int s = burst_serial_try_send(st_ring + t, k);
        if (s <= 0) break;
        st_tail += s;
    }
}
/* Queue header + payload as one unit; false (nothing queued) when it does not fit. */
static bool ring_frame(const char *h, unsigned hl, const void *pl, unsigned bytes) {
    if (ring_used() + hl + bytes > st_ring_size) return false;
    ring_put(h, hl); if (bytes) ring_put(pl, bytes);
    ring_pump();
    return true;
}
static bool ring_open_sz(unsigned size) {   /* size: power of two */
    st_ring = heap_caps_malloc(size, MALLOC_CAP_8BIT); st_ring_size = size;
    if (!st_ring) return false;
    st_head = st_tail = 0;
    st_isr = burst_serial_ring_start(st_ring, size, &st_head, &st_tail);
    return true;
}
static bool ring_open(void) { return ring_open_sz(STREAM_RING); }
static void ring_close(void) {
    int64_t lim = esp_timer_get_time() + 3000000;
    while (ring_used() && esp_timer_get_time() < lim) ring_pump();
    if (st_isr) { burst_serial_ring_stop(); st_isr = false; }
    free(st_ring); st_ring = NULL;
}

/* ------------------------------------------------- continuous engine */
/* Measured (diagnostics/c5_gap_probe.h): neither the CPU nor the GDMA can read
 * bank 1 while the dump owns it (GDMATEST: reads return stale words), so true
 * ping-pong is impossible.  But a finished dump re-arms in ~4 us (B2B, 99.9 % duty with no
 * work) and memcpy of a block into the heap costs ~2.6 CPU cycles per word.
 * So the engine dumps block k+1 while the CPU works on the heap copy of block
 * k; the only gap per block is the copy (~90 us per 8192 words): 96 % duty at
 * 4 MS/s, 92 % at 8 MS/s, 83 % at 20 MS/s.  Gaps are counted exactly with the
 * CPU cycle counter (CPU and dump clock share the crystal: 3/6/12/24/30/60
 * cycles per sample).  CGRID schedules block starts on a time grid so two
 * boards can interleave and cover each other's gaps (CADJ trims it live). */
#define CONT_WORDS 8192u
static const uint8_t hw_cps[6] = {3, 6, 12, 24, 30, 60};
typedef struct {
    unsigned div, n, cps;
    uint32_t *buf;                       /* heap copy of the last finished block */
    uint32_t owner; unsigned analog[2];
    uint32_t cyc_go, cyc_prev; int64_t t_go; bool have_prev;
    int64_t grid_next; uint32_t grid_period;
    int64_t blk_t0; uint32_t blk_gap;    /* block in buf: time of sample 0, input samples missing before it */
    uint32_t blocks, slips, late, maxgap;
    uint64_t lost, copy_cyc, wait_cyc, work_cyc;
} cont_t;
static int64_t cgrid_t0; static uint32_t cgrid_period;   /* CGRID: schedule of the next run */
static cont_t cont_last;                                  /* CSTAT? */

static void IRAM_ATTR cont_go(cont_t *c) {
    if (c->grid_period) {
        int64_t now = esp_timer_get_time();
        if (now > c->grid_next + c->grid_period / 4) {       /* hopelessly late: skip slots */
            int64_t k = (now - c->grid_next) / c->grid_period + 1;
            c->grid_next += k * c->grid_period; c->slips += (uint32_t)k;
        } else if (now > c->grid_next) c->late++;
        while (esp_timer_get_time() < c->grid_next) {}
        c->grid_next += c->grid_period;
    }
    dump_arm(c->div, c->n, false, &c->owner);
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    c->t_go = esp_timer_get_time(); c->cyc_go = esp_cpu_get_cycle_count();
    dump_go();
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
}
static void cont_start(cont_t *c, unsigned div, uint32_t *buf, unsigned words) {
    memset(c, 0, sizeof(*c));
    c->div = div; c->n = words; c->cps = hw_cps[div]; c->buf = buf;
    c->grid_period = cgrid_period;
    c->grid_next = cgrid_t0 ? cgrid_t0 : esp_timer_get_time() + 2000;
    prepare_rx(); rx_analog_apply(c->analog);
    cont_go(c);
}
/* Wait for the running dump, copy it to c->buf and start the next one. */
static bool cont_next(cont_t *c) {
    uint32_t w0 = esp_cpu_get_cycle_count();
    int64_t lim = esp_timer_get_time() + 100000;
    while (!(REG_READ(DUMP_CTRL) & (1u << 18))) { ring_pump(); if (esp_timer_get_time() > lim) return false; }
    uint32_t w1 = esp_cpu_get_cycle_count();
    dump_halt(c->owner);
    memcpy(c->buf, DUMP_BUF, c->n * 4);
    uint32_t cg = c->cyc_go; int64_t tg = c->t_go;
    cont_go(c);
    c->wait_cyc += w1 - w0; c->copy_cyc += esp_cpu_get_cycle_count() - w1;
    c->blk_t0 = tg; c->blk_gap = 0;
    if (c->have_prev) {
        uint32_t span = (cg - c->cyc_prev + c->cps / 2) / c->cps;   /* samples between the two triggers */
        c->blk_gap = span > c->n ? span - c->n : 0;
    }
    c->cyc_prev = cg; c->have_prev = true;
    c->lost += c->blk_gap; if (c->blk_gap > c->maxgap) c->maxgap = c->blk_gap;
    c->blocks++;
    return true;
}
static void cont_stop(cont_t *c) {
    int64_t lim = esp_timer_get_time() + 100000;
    while (!(REG_READ(DUMP_CTRL) & (1u << 18)) && esp_timer_get_time() < lim) {}
    dump_halt(c->owner);
    rx_analog_restore(c->analog);
    cont_last = *c;
}
/* Lines during a run: "CADJ <+-us>" shifts the grid, anything else stops. */
static bool cont_poll(cont_t *c) {
    char l[48]; int32_t d; char x;
    while (burst_serial_poll_run_line(l, sizeof(l))) {
        if (sscanf(l, "CADJ %" SCNd32 " %c", &d, &x) == 1) { if (c->grid_period) c->grid_next += d; continue; }
        return true;
    }
    return false;
}

static uint32_t st_cyc[5], st_blocks;   /* per block: wait, cic, fir, frame, work total */
/* Continuous block stream: every block (8192 samples at the hardware rate)
 * goes through CIC3 + half-band/Kaiser FIR with filter and NCO state kept
 * across blocks.  Frames: "BLK seq idx n crc t_us gap_in"; gap_in = input
 * samples missing before the frame (block gap, start-up warm-up). */
static bool stream_run(unsigned bits, unsigned div, unsigned R1, unsigned R2, int32_t offset_hz, uint64_t limit, bool dcblock) {
    dsp_init_tables();
    memset(st_cyc, 0, sizeof(st_cyc)); st_blocks = 0;
    static cic3_t cic; static firdec_t *fd;
    /* block copy first (largest); halve it once if the rest does not fit */
    bool ring = false; int32_t *mid = NULL, *outb = NULL; uint32_t *cbuf = NULL; unsigned words = CONT_WORDS;
    for (; words >= CONT_WORDS / 2; words /= 2) {
        cbuf = heap_caps_malloc(words * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ring = cbuf && ring_open();
        fd = heap_caps_malloc(sizeof(firdec_t), MALLOC_CAP_8BIT);
        mid = heap_caps_malloc(1032 * 4 + 64, MALLOC_CAP_8BIT);              /* CIC out of one chunk / payload */
        outb = heap_caps_malloc(2 * 4 * (1024 + 8), MALLOC_CAP_8BIT);        /* FIR out of one frame */
        if (cbuf && ring && fd && mid && outb) break;
        if (ring) ring_close();
        free(cbuf); free(fd); free(mid); free(outb); cbuf = NULL; ring = false;
    }
    if (!cbuf) {
        char e[64]; snprintf(e, sizeof(e), "ERR stream_memory %u\n", (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        reply(e); return false;
    }
    unsigned fin = hw_rates[div], fm = fin / R1, out_rate = fm / R2;
    unsigned gbits = 0; while ((1u << gbits) < R1 * R1 * R1) gbits++;
    unsigned taps = 16 * R2 + 1; if (taps > 129) taps = 129;
    fir_design(taps, 0.5f / R2, 7.0f);
    unsigned warm = taps / R2 + 2;                     /* filter start-up outputs dropped once */
    uint32_t ph = 0, dph = (uint32_t)(int32_t)llroundf((float)offset_hz / fm * 4294967296.0f);
    int32_t dci = 0, dcq = 0;
    char h[160];
    snprintf(h, sizeof(h), "OK STREAM %u %u %u %u %u %u %u\n", out_rate, bits, fin, R1, R2, taps, words);
    reply(h);
    memset(&cic, 0, sizeof(cic)); cic.R = R1; cic.sh = gbits + 9 > 14 ? gbits + 9 - 14 : 0;
    memset(fd, 0, sizeof(*fd)); fd->R = R2;
    static cont_t c;
    cont_start(&c, div, cbuf, words);
    uint64_t out_idx = 0, produced = 0; uint32_t seq = 0, overruns = 0, pending_gap = 0;
    bool stop = false; const char *why = "stop";
    int64_t last_progress = esp_timer_get_time(); unsigned last_tail = 0;
    while (!stop) {
        /* keep the ring from overflowing: a late halt only lengthens the reported gap */
        while (!stop && ring_used() > STREAM_RING / 2) {
            ring_pump();
            if (cont_poll(&c)) stop = true;
            if (st_tail != last_tail) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
            if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; stop = true; }
        }
        if (stop) break;
        uint32_t c0 = esp_cpu_get_cycle_count();
        if (!cont_next(&c)) { why = "capture_error"; break; }
        uint32_t c1 = esp_cpu_get_cycle_count(); st_cyc[0] += c1 - c0;
        uint32_t gap_in = c.blk_gap;
        if (gap_in) ph += (uint32_t)(((int64_t)(int32_t)dph * gap_in) / R1);  /* NCO phase-coherent across the gap */
        pending_gap += gap_in;
        /* input index (within this block) completing the block's first output */
        uint32_t i0 = (R1 - cic.k - 1) + R1 * (R2 - fd->k - 1);
        unsigned nout = 0, frame_first = 0;
        for (unsigned off = 0; off < words && !stop; off += 2048) {
            unsigned m = words - off > 2048 ? 2048 : words - off;
            uint32_t d1 = esp_cpu_get_cycle_count();
            unsigned k = cic3_run(&cic, cbuf + off, m, mid);
            for (unsigned j = 0; j < k; j++) {
                int32_t I = mid[2 * j], Q = mid[2 * j + 1];
                if (dcblock) { dci += ((I << 8) - dci) >> 10; dcq += ((Q << 8) - dcq) >> 10; I -= dci >> 8; Q -= dcq >> 8; }
                if (dph) {
                    int32_t cs = sin_tab[((ph >> 22) + 256) & 1023], sn = sin_tab[ph >> 22];
                    int32_t I2 = (I * cs - Q * sn) >> 14, Q2 = (I * sn + Q * cs) >> 14; I = I2; Q = Q2; ph += dph;
                }
                mid[2 * j] = I; mid[2 * j + 1] = Q;
            }
            uint32_t d2 = esp_cpu_get_cycle_count(); st_cyc[1] += d2 - d1;
            nout += firdec_run(fd, mid, k, outb + 2 * nout);
            st_cyc[2] += esp_cpu_get_cycle_count() - d2;
            ring_pump();
            if (nout < 768 && off + m < words) continue;
            uint32_t e0 = esp_cpu_get_cycle_count();
            unsigned skip = produced < warm ? (unsigned)(warm - produced < nout ? warm - produced : nout) : 0;
            produced += nout;
            pending_gap += skip * R1 * R2;
            unsigned n = nout - skip;
            if (limit && out_idx + n > limit) n = (unsigned)(limit - out_idx);
            unsigned bytes = n * (bits / 4);
            uint8_t *pl = (uint8_t *)mid;
            for (unsigned j = 0; j < n; j++) {
                int32_t I = outb[2 * (j + skip)], Q = outb[2 * (j + skip) + 1];
                if (bits == 16) {
                    I = I > 32767 ? 32767 : I < -32768 ? -32768 : I; Q = Q > 32767 ? 32767 : Q < -32768 ? -32768 : Q;
                    ((int16_t *)pl)[2 * j] = I; ((int16_t *)pl)[2 * j + 1] = Q;
                } else {
                    I >>= 7; Q >>= 7;
                    ((int8_t *)pl)[2 * j] = I > 127 ? 127 : I < -128 ? -128 : I; ((int8_t *)pl)[2 * j + 1] = Q > 127 ? 127 : Q < -128 ? -128 : Q;
                }
            }
            uint64_t first_in = i0 + (uint64_t)(frame_first + skip) * R1 * R2;
            int64_t t_first = c.blk_t0 + (int64_t)(first_in * 1000000u / fin);
            if (n) {
                uint32_t crc = esp_rom_crc32_le(0, pl, bytes);
                int hl = snprintf(h, sizeof(h), "BLK %" PRIu32 " %" PRIu64 " %u %08" PRIx32 " %" PRId64 " %" PRIu32 "\n",
                                  seq, out_idx, n, crc, t_first, pending_gap);
                if (!ring_frame(h, hl, pl, bytes)) overruns++;   /* transport too slow: frame dropped, seq/idx gap */
                seq++; out_idx += n; pending_gap = 0;
            }
            frame_first += nout; nout = 0;
            st_cyc[3] += esp_cpu_get_cycle_count() - e0;
            if (limit && out_idx >= limit) { why = "limit"; stop = true; }
            if (cont_poll(&c)) stop = true;
        }
        uint32_t c2 = esp_cpu_get_cycle_count();
        st_cyc[4] += c2 - c1; c.work_cyc += c2 - c1; st_blocks++;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    cont_stop(&c);
    ring_close();
    snprintf(h, sizeof(h), "END STREAM %" PRIu32 " %" PRIu64 " %" PRIu64 " %" PRIu32 " %s\n", seq, out_idx, c.lost, overruns, why);
    burst_serial_drain_stop();
    reply(h);
    free(cbuf); free(fd); free(mid); free(outb);
    return true;
}

/* ------------------------------------------ continuous spectrogram (CSPEC) */
/* Every sample of every block goes through a Hann-windowed FFT (nfft 16-256);
 * navg FFTs form one line, reported as mean and/or max-hold power (a short
 * packet inside a line stays visible in the max line).  Lines never span
 * blocks (nfft*navg divides the block).  Line value u8 = 2*10*log10(|X|^2)
 * (0.5 dB steps), X = FFT of hann*x with x in 10-bit LSB; host orientation
 * (-fs/2..+fs/2, host negates Q) like SPEC. */
/* The CSPEC DSP runs from flash (cache): measured as fast as IRAM in CSPEC,
 * and ~8 KB of IRAM would shrink the largest heap block below the 32 KB
 * block copy + 16 KB ring (-> 4096-word blocks, longer relative gaps). */
#define CS_ATTR
#include "cspec_dsp.h"
static uint32_t cs_cyc[3];   /* per block: fft, lines, frame */
/* One line: navg segments (stride*nfft apart) -> nfft u8 bins (mode 0 mean,
 * 1 max) or mean then max (mode 2) at o.  acc: u64 sums (only when navg >
 * CS_FOLD), a32: u32 partial sums, mx: max; all in FFT output order. */
static void CS_ATTR cs_line(const uint32_t *w, unsigned nfft, unsigned navg, unsigned stride, unsigned mode, unsigned sh,
                              int32_t *x, uint64_t *acc, uint32_t *a32, uint32_t *mx, uint8_t *o, uint32_t *fcyc) {
    uint32_t c0 = esp_cpu_get_cycle_count();
    bool wide = navg > CS_FOLD;
    if (mode != 1) { memset(a32, 0, nfft * 4); if (wide) memset(acc, 0, nfft * 8); }
    if (mode != 0) memset(mx, 0, nfft * 4);
    for (unsigned sg = 0, fold = 0; sg < navg; sg++) {
        cs_segment(w + sg * stride * nfft, x, a32, mx, nfft, sh, mode);
        if (wide && mode != 1 && (++fold == CS_FOLD || sg == navg - 1)) {
            for (unsigned j = 0; j < nfft; j++) { acc[j] += a32[j]; a32[j] = 0; }
            fold = 0;
        }
    }
    *fcyc += esp_cpu_get_cycle_count() - c0;
    unsigned lgn = __builtin_ctz(navg);
    uint8_t *om = mode == 2 ? o + nfft : o;
    for (unsigned hb = 0; hb < nfft; hb++) {
        unsigned p = cs_hpos[hb];
        if (mode != 1) o[hb] = cs_u8f(wide ? (uint32_t)(acc[p] >> lgn) : a32[p] >> lgn, sh);
        if (mode != 0) om[hb] = cs_u8f(mx[p], sh);
    }
}
static bool cspec_run(unsigned div, unsigned nfft, unsigned navg, unsigned mode, unsigned stride) {
    cs_init(nfft);
    unsigned sh = nfft > 64 ? __builtin_ctz(nfft) - 6 : 0;
    unsigned bpp = mode == 2 ? 2 : 1, per_line = nfft * navg * stride, words = CONT_WORDS;
    bool ring = false; int32_t *x = NULL; uint64_t *acc = NULL; uint32_t *mx, *a32; uint8_t *fb = NULL; uint32_t *cbuf = NULL;
    for (; words >= CONT_WORDS / 2 && words >= per_line; words /= 2) {
        cbuf = heap_caps_malloc(words * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ring = cbuf && ring_open();
        x = heap_caps_malloc(nfft * 8, MALLOC_CAP_8BIT);
        acc = heap_caps_malloc(nfft * 16, MALLOC_CAP_8BIT);
        fb = heap_caps_malloc((words / per_line) * nfft * bpp, MALLOC_CAP_8BIT);
        if (cbuf && ring && x && acc && fb) break;
        if (ring) ring_close();
        free(cbuf); free(x); free(acc); free(fb); cbuf = NULL; ring = false;
    }
    if (!cbuf) { reply("ERR cspec_memory\n"); return false; }
    mx = (uint32_t *)(acc + nfft); a32 = mx + nfft;
    unsigned lines = words / per_line, fbytes = lines * nfft * bpp;
    char h[128];
    snprintf(h, sizeof(h), "OK CSPEC %u %u %u %u %u %u\n", hw_rates[div], nfft, navg, mode, words, lines);
    reply(h);
    memset(cs_cyc, 0, sizeof(cs_cyc));
    static cont_t c;
    cont_start(&c, div, cbuf, words);
    uint32_t seq = 0, overruns = 0; bool stop = false; const char *why = "stop";
    int64_t last_progress = esp_timer_get_time(); unsigned last_tail = 0;
    while (!stop) {
        while (!stop && ring_used() > STREAM_RING / 2) {
            ring_pump();
            if (cont_poll(&c)) stop = true;
            if (st_tail != last_tail) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
            if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; stop = true; }
        }
        if (stop) break;
        if (!cont_next(&c)) { why = "capture_error"; break; }
        uint32_t c1 = esp_cpu_get_cycle_count(), f = 0, l = 0;
        for (unsigned ln = 0; ln < lines; ln++) {
            uint32_t a0 = esp_cpu_get_cycle_count();
            cs_line(cbuf + ln * per_line, nfft, navg, stride, mode, sh, x, acc, a32, mx, fb + ln * nfft * bpp, &f);
            l += esp_cpu_get_cycle_count() - a0;
            ring_pump();
        }
        l -= f;
        uint32_t e0 = esp_cpu_get_cycle_count();
        uint32_t crc = esp_rom_crc32_le(0, fb, fbytes);
        int hl = snprintf(h, sizeof(h), "SPL %" PRIu32 " %" PRId64 " %u %u %u %" PRIu32 " %08" PRIx32 "\n",
                          seq, c.blk_t0, lines, nfft, mode, c.blk_gap, crc);
        if (!ring_frame(h, hl, fb, fbytes)) overruns++;
        seq++;
        uint32_t e1 = esp_cpu_get_cycle_count();
        cs_cyc[0] += f; cs_cyc[1] += l; cs_cyc[2] += e1 - e0; c.work_cyc += e1 - c1;
        if (cont_poll(&c)) stop = true;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    cont_stop(&c);
    ring_close();
    snprintf(h, sizeof(h), "END CSPEC %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu32 " %s\n", seq, c.blocks, c.lost, overruns, why);
    burst_serial_drain_stop();
    reply(h);
    free(cbuf); free(x); free(acc); free(fb);
    return true;
}

/* ------------------------------------------------ burst capture (CPKT) */
/* Gap-minimised burst catcher on the continuous engine: per block the
 * magnitude m = mean(|I|+|Q|)/2 over dec samples is formed for every sample,
 * a burst triggers when m > floor*thr/10 (floor = EMA of the block means) and
 * pre+len magnitudes around it are sent as "PKT seq t_us n dec crc flags"
 * + n x u8 (flags 1 = cut at the block end).  Magnitude only, so a PLL offset
 * of a few MHz inside the capture bandwidth does not matter (OOK/ASK/PPM bursts). */
static void __attribute__((noinline)) IRAM_ATTR pk_mags(const uint32_t *restrict w, unsigned n, unsigned dec, unsigned lg, uint8_t *restrict m, uint32_t *sum) {
    uint32_t tot = 0;
    for (unsigned j = 0; j < n; j += dec) {
        uint32_t a = 0;
        for (unsigned k = 0; k < dec; k++) {
            uint32_t v = w[j + k];
            int32_t I = ((int32_t)(v << 22)) >> 22, Q = ((int32_t)(v << 12)) >> 22;
            a += (uint32_t)((I ^ (I >> 31)) - (I >> 31)) + (uint32_t)((Q ^ (Q >> 31)) - (Q >> 31));
        }
        a >>= lg + 1; tot += a;
        *m++ = a > 255 ? 255 : (uint8_t)a;
    }
    *sum = tot;
}
static bool cpkt_run(unsigned div, unsigned dec, unsigned thr, unsigned pre, unsigned len) {
    unsigned lg = __builtin_ctz(dec), words = CONT_WORDS;
    bool ring = false; uint8_t *mag = NULL; uint32_t *cbuf = NULL;
    for (; words >= CONT_WORDS / 2; words /= 2) {
        cbuf = heap_caps_malloc(words * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ring = cbuf && ring_open();
        mag = heap_caps_malloc(words / dec, MALLOC_CAP_8BIT);
        if (cbuf && ring && mag) break;
        if (ring) ring_close();
        free(cbuf); free(mag); cbuf = NULL; ring = false;
    }
    if (!cbuf) { reply("ERR cpkt_memory\n"); return false; }
    unsigned nm = words / dec, fin = hw_rates[div];
    char h[128];
    snprintf(h, sizeof(h), "OK CPKT %u %u %u %u %u %u\n", fin, dec, thr, pre, len, words);
    reply(h);
    static cont_t c;
    cont_start(&c, div, cbuf, words);
    uint32_t seq = 0, overruns = 0, floor = 0; uint64_t pk_total = 0; bool stop = false; const char *why = "stop";
    int64_t last_progress = esp_timer_get_time(); unsigned last_tail = 0;
    while (!stop) {
        while (!stop && ring_used() > STREAM_RING / 2) {
            ring_pump();
            if (cont_poll(&c)) stop = true;
            if (st_tail != last_tail) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
            if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; stop = true; }
        }
        if (stop) break;
        if (!cont_next(&c)) { why = "capture_error"; break; }
        uint32_t c1 = esp_cpu_get_cycle_count(), sum;
        pk_mags(cbuf, words, dec, lg, mag, &sum);
        uint32_t mean16 = (sum << 4) / nm;                       /* block mean, 1/16 LSB */
        floor = floor ? (7 * floor + mean16) / 8 : mean16;
        uint32_t trig = (floor * thr / 10 + 15) / 16; if (trig < 2) trig = 2;
        for (unsigned j = 0; j < nm; j++) {
            if (mag[j] <= trig) continue;
            unsigned a = j > pre ? j - pre : 0, e = a + pre + len, flags = 0;
            if (e > nm) { e = nm; flags = 1; }
            unsigned n = e - a;
            uint32_t crc = esp_rom_crc32_le(0, mag + a, n);
            int64_t t = c.blk_t0 + (int64_t)((uint64_t)a * dec * 1000000u / fin);
            int hl = snprintf(h, sizeof(h), "PKT %" PRIu32 " %" PRId64 " %u %u %08" PRIx32 " %u %u\n", seq, t, n, dec, crc, flags, (unsigned)mag[j]);
            if (!ring_frame(h, hl, mag + a, n)) overruns++;
            seq++; pk_total++;
            j = e - 1;                                            /* hold off until the window end */
        }
        c.work_cyc += esp_cpu_get_cycle_count() - c1;
        if (cont_poll(&c)) stop = true;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    cont_stop(&c);
    ring_close();
    snprintf(h, sizeof(h), "END CPKT %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %s\n", seq, c.blocks, c.lost, overruns, floor, why);
    burst_serial_drain_stop();
    reply(h);
    free(cbuf); free(mag);
    return true;
}

/* ------------------------------------------------------------ commands */
/* Pick hardware rate + CIC/FIR split for a stream output rate (8 MS/s by
 * default: better alias protection than 4 MS/s, see docs/espsoup-c5.md). */
static bool stream_plan(unsigned out, unsigned hwi, unsigned *div, unsigned *R1, unsigned *R2) {
    static const unsigned pref[4] = {4, 3, 2, 5};  /* 8, 10, 20, 4 MS/s */
    for (unsigned r2 = 2; r2 <= 8; r2 *= 2)
        for (unsigned f = 0; f < 4; f++) {
            unsigned d = hwi <= 5 ? hwi : pref[f];
            unsigned fin = hw_rates[d];
            if (fin % (out * r2) == 0) {
                unsigned r1 = fin / (out * r2);
                if (r1 >= 4 && r1 <= 128) { *div = d; *R1 = r1; *R2 = r2; return true; }
            }
            if (hwi <= 5) break;
        }
    return false;
}

static bool cont_command(const char *line) {
    unsigned rate, bits = 0; char extra; int32_t off = 0; int m;
    /* CGRID <t0_us> <period_us>: block starts of the next continuous run at
     * t0 + k*period (board clock, TIME?); CGRID 0 0 = free-running. */
    uint32_t per; int64_t t0g;
    if (sscanf(line, "CGRID %" SCNd64 " %" SCNu32 " %c", &t0g, &per, &extra) == 2) {
        if (per && per < 200) { reply("ERR cgrid_args\n"); return true; }
        cgrid_t0 = per ? t0g : 0; cgrid_period = per; reply("OK\n"); return true;
    }
    if (!strcmp(line, "CSTAT?")) {
        const cont_t *c = &cont_last; char h[220]; unsigned b = c->blocks ? c->blocks : 1;
        double duty = c->blocks ? (double)c->blocks * c->n / ((double)c->blocks * c->n + (double)c->lost) : 0;
        snprintf(h, sizeof(h), "CSTAT blocks %" PRIu32 " words %u lost %" PRIu64 " maxgap %" PRIu32 " duty %.4f slips %" PRIu32 " late %" PRIu32
                 " copy %" PRIu64 " wait %" PRIu64 " work %" PRIu64 " fft %" PRIu32 " lines %" PRIu32 "\n",
                 c->blocks, c->n, c->lost, c->maxgap, duty, c->slips, c->late, c->copy_cyc / b, c->wait_cyc / b, c->work_cyc / b,
                 cs_cyc[0] / b, cs_cyc[1] / b);
        reply(h); return true;
    }
    /* CSPEC <rate_idx> <nfft> <navg> [<mode> [<stride>]]: continuous spectrogram, mode 0 mean, 1 max, 2 both */
    unsigned cn, ca, cm = 1, cst = 1; int cmn;
    if ((cmn = sscanf(line, "CSPEC %u %u %u %u %u %c", &rate, &cn, &ca, &cm, &cst, &extra)) >= 3 && cmn <= 5) {
        if (rate > 5 || cn < 16 || cn > CS_NMAX || (cn & (cn - 1)) || !ca || (ca & (ca - 1)) || !cst || (cst & (cst - 1)) || cn * ca * cst > CONT_WORDS / 2 || cm > 2 ||
            (CONT_WORDS / (cn * ca)) * cn * (cm == 2 ? 2 : 1) > 8192) { reply("ERR cspec_args\n"); return true; }
        cspec_run(rate, cn, ca, cm, cst); return true;
    }
    /* CPKT <rate_idx> <dec> <thr_x10> <pre> <len>: burst catcher (magnitudes) */
    unsigned pd, pt, pp, pl;
    if (sscanf(line, "CPKT %u %u %u %u %u %c", &rate, &pd, &pt, &pp, &pl, &extra) == 5) {
        if (rate > 5 || !pd || pd > 16 || (pd & (pd - 1)) || pt < 11 || pt > 1000 || pp > 256 || !pl || pl > 4096) { reply("ERR cpkt_args\n"); return true; }
        cpkt_run(rate, pd, pt, pp, pl); return true;
    }
    /* STREAM8/STREAM16 <out_rate_hz> [<offset_hz> [<samples> [<dcblock> [<hw_rate_idx>]]]] */
    uint32_t lim32 = 0; unsigned dcb = 1, hwi = 99;
    if (((m = sscanf(line, "STREAM8 %u %" SCNd32 " %" SCNu32 " %u %u %c", &rate, &off, &lim32, &dcb, &hwi, &extra)) >= 1 && m <= 5 && (bits = 8)) ||
        ((m = sscanf(line, "STREAM16 %u %" SCNd32 " %" SCNu32 " %u %u %c", &rate, &off, &lim32, &dcb, &hwi, &extra)) >= 1 && m <= 5 && (bits = 16))) {
        if (m < 2) off = 0;
        if (m < 3) lim32 = 0;
        if (m < 4) dcb = 1;
        unsigned div, R1, R2;
        bool ok = rate >= 1000 && rate <= 500000 && stream_plan(rate, hwi, &div, &R1, &R2);
        if (!ok || off > (int32_t)(hw_rates[div] / R1 / 4) || off < -(int32_t)(hw_rates[div] / R1 / 4)) {
            reply("ERR stream_args\n"); return true;
        }
        stream_run(bits, div, R1, R2, off, lim32, dcb != 0);
        return true;
    }
    if (!strcmp(line, "STREAMSTAT?")) {
        char h[160]; unsigned d = st_blocks ? st_blocks : 1;
        snprintf(h, sizeof(h), "STREAMSTAT %" PRIu32 " capture %" PRIu32 " cic %" PRIu32 " fir %" PRIu32 " frame %" PRIu32 " total %" PRIu32 "\n",
                 st_blocks, st_cyc[0] / d, st_cyc[1] / d, st_cyc[2] / d, st_cyc[3] / d, st_cyc[4] / d);
        reply(h); return true;
    }
    /* CSBENCH <nfft> <navg> <mode> (diagnostic): CSPEC cost on a heap copy of the
     * last dump, CPU cycles per input sample: line total, segments, and the FFT
     * parts (first + middle stages, last stage + power). */
    unsigned bn, ba, bm;
    if (sscanf(line, "CSBENCH %u %u %u %c", &bn, &ba, &bm, &extra) == 3) {
        if (bn < 16 || bn > CS_NMAX || (bn & (bn - 1)) || !ba || (ba & (ba - 1)) || bn * ba > 4096 || bm > 2) { reply("ERR csbench_args\n"); return true; }
        unsigned ns = bn * ba, sh = bn > 64 ? __builtin_ctz(bn) - 6 : 0;
        uint32_t *w = heap_caps_malloc(ns * 4, MALLOC_CAP_8BIT); int32_t *x = heap_caps_malloc(bn * 8, MALLOC_CAP_8BIT);
        uint64_t *acc = heap_caps_malloc(bn * 16, MALLOC_CAP_8BIT); uint8_t *o = heap_caps_malloc(2 * bn, MALLOC_CAP_8BIT);
        if (!w || !x || !acc || !o) { free(w); free(x); free(acc); free(o); reply("ERR mem\n"); return true; }
        uint32_t *mx = (uint32_t *)(acc + bn), *a32 = mx + bn;
        dsp_init_tables(); cs_init(bn);
        memcpy(w, DUMP_BUF, ns * 4);
        uint32_t f = 0, c0 = esp_cpu_get_cycle_count();
        for (unsigned r = 0; r < 4; r++) cs_line(w, bn, ba, 1, bm, sh, x, acc, a32, mx, o, &f);
        uint32_t c1 = esp_cpu_get_cycle_count();
        for (unsigned s = 0; s < ba; s++) cs_front(w + s * bn, x, bn);
        uint32_t c2 = esp_cpu_get_cycle_count();
        for (unsigned s = 0; s < ba; s++) cs_last(x, bn, a32, mx, bm);
        uint32_t c3 = esp_cpu_get_cycle_count();
        char h[200];
        snprintf(h, sizeof(h), "CSBENCH %u %u %u line %.1f seg %.1f front %.1f last %.1f u8 %u %u %u\n", bn, ba, bm,
                 (c1 - c0) / 4.0 / ns, f / 4.0 / ns, (double)(c2 - c1) / ns, (double)(c3 - c2) / ns, o[0], o[bn / 2], o[bn - 1]);
        free(w); free(x); free(acc); free(o); reply(h); return true;
    }
    /* CSDATA <rate_idx> <nfft> <navg> <mode> (diagnostic): one snapshot of
     * nfft*navg*L <= 16380 samples -> "DATAT" frame (CAPT20 payload) followed by
     * "CSL lines nfft mode crc" + the CSPEC lines computed on exactly these
     * samples (DSP check on real data). */
    if (sscanf(line, "CSDATA %u %u %u %u %c", &rate, &bn, &ba, &bm, &extra) == 4) {
        if (rate > 5 || bn < 16 || bn > CS_NMAX || (bn & (bn - 1)) || !ba || (ba & (ba - 1)) || bn * ba > 4096 || bm > 2) { reply("ERR csdata_args\n"); return true; }
        unsigned per_line = bn * ba, lines = 16380 / per_line, ns = lines * per_line, bpp = bm == 2 ? 2 : 1, sh = bn > 64 ? __builtin_ctz(bn) - 6 : 0;
        int32_t *x = heap_caps_malloc(bn * 8, MALLOC_CAP_8BIT); uint64_t *acc = heap_caps_malloc(bn * 16, MALLOC_CAP_8BIT);
        uint8_t *o = heap_caps_malloc(lines * bn * bpp, MALLOC_CAP_8BIT);
        if (!x || !acc || !o) { free(x); free(acc); free(o); reply("ERR mem\n"); return true; }
        uint32_t *mx = (uint32_t *)(acc + bn), *a32 = mx + bn, f = 0, el;
        dsp_init_tables(); cs_init(bn); prepare_rx();
        const char *err = fast_capture(ns, rate, &el);
        if (!err) {
            for (unsigned l = 0; l < lines; l++) cs_line(DUMP_BUF + l * per_line, bn, ba, 1, bm, sh, x, acc, a32, mx, o + l * bn * bpp, &f);
            opt_pack(DUMP_BUF, ns, 20);
            if (opt_send_frame(DUMP_BUF, opt_wire(ns, 20), ns, el, true)) {
                char h[96]; snprintf(h, sizeof(h), "CSL %u %u %u %08" PRIx32 "\n", lines, bn, bm, esp_rom_crc32_le(0, o, lines * bn * bpp));
                (void)(send_bytes(h, strlen(h)) && send_bytes(o, lines * bn * bpp));
            }
        } else reply(err);
        free(x); free(acc); free(o); return true;
    }
    return false;
}
