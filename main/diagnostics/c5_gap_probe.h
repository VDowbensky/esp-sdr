/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESP32-C5 dump-bank gap investigation (-DC5_GAP_PROBE=ON builds only; never packaged).
 * GDMATEST: can the GDMA read the dump bank while the modem owns it?
 * RATETEST: what do dump dividers 6/7 produce?
 * B2B:      back-to-back capture period / duty with a given per-block work. */
#pragma once
#include "esp_async_memcpy.h"

static async_memcpy_handle_t gp_mcp;
static volatile bool gp_done;
static bool gp_cb(async_memcpy_handle_t h, async_memcpy_event_t *e, void *a) { (void)h; (void)e; (void)a; gp_done = true; return false; }

/* GDMATEST <div> <words> <mode>
 * mode 0: DMA copy of bank[0..words) while the free-running dump writes the
 *         lower half (copy starts once the writer passed 8192 > words).
 * mode 1: same copy after the dump was halted (control).
 * Reply: GDMA <same> <dmasent> <bankfresh> <dmafresh> <dma_us> <pos_start> <pos_end> <w0 dma/bank ...> */
static void gp_gdmatest(unsigned div, unsigned words, unsigned mode) {
    prepare_rx();
    if (!gp_mcp) {
        async_memcpy_config_t cfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
        cfg.backlog = 4;
        if (esp_async_memcpy_install_gdma_ahb(&cfg, &gp_mcp) != ESP_OK) { reply("ERR gdma_install\n"); return; }
    }
    uint32_t *dst = heap_caps_aligned_alloc(16, words * 4, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!dst) { reply("ERR mem\n"); return; }
    for (unsigned j = 0; j < words; j++) dst[j] = 0x11111111u;
    volatile uint32_t *w = DUMP_WORDS;
    for (unsigned j = 0; j < 2 * HALF_WORDS; j++) w[j] = SENTINEL;
    uint32_t owner, ps = 0, pe = 0; int64_t t1 = 0, t2 = 0; esp_err_t er = ESP_OK;
    dump_arm(div, 0x1ffffu, false, &owner);
    dump_go();
    int64_t lim = esp_timer_get_time() + 50000;
    while ((dump_pos16() & 0x3fffu) < 8192 && esp_timer_get_time() < lim) {}
    if (mode == 0) {
        gp_done = false; ps = dump_pos16(); t1 = esp_timer_get_time();
        er = esp_async_memcpy(gp_mcp, dst, (void *)DUMP_BUF, words * 4, gp_cb, NULL);
        while (!gp_done && esp_timer_get_time() - t1 < 20000) {}
        t2 = esp_timer_get_time(); pe = dump_pos16();
        dump_halt(owner);
    } else {
        dump_halt(owner);
        gp_done = false; t1 = esp_timer_get_time();
        er = esp_async_memcpy(gp_mcp, dst, (void *)DUMP_BUF, words * 4, gp_cb, NULL);
        while (!gp_done && esp_timer_get_time() - t1 < 20000) {}
        t2 = esp_timer_get_time();
    }
    unsigned same = 0, dsent = 0, bfresh = 0, dfresh = 0;
    for (unsigned j = 0; j < words; j++) {
        uint32_t b = w[j], d = dst[j];
        same += b == d; dsent += d == SENTINEL || d == 0x11111111u;
        bfresh += b != SENTINEL; dfresh += d != SENTINEL && d != 0x11111111u;
    }
    char h[240];
    int o = snprintf(h, sizeof(h), "GDMA er %d done %d same %u dmasent %u bankfresh %u dmafresh %u us %" PRId64 " pos %" PRIu32 " %" PRIu32,
                     (int)er, (int)gp_done, same, dsent, bfresh, dfresh, t2 - t1, ps, pe);
    for (unsigned j = 0; j < 3; j++) o += snprintf(h + o, sizeof(h) - o, " %08" PRIx32 "/%08" PRIx32, dst[100 + j * 1000], w[100 + j * 1000]);
    snprintf(h + o, sizeof(h) - o, "\n");
    free(dst); reply(h);
}

/* RATETEST <div> <n>: finite dump with any divider (0-7), report elapsed us
 * (timeout 300 ms) and how many words were written. */
static void gp_ratetest(unsigned div, unsigned n) {
    prepare_rx();
    volatile uint32_t *w = DUMP_WORDS;
    for (unsigned j = 0; j < n; j++) w[j] = SENTINEL;
    uint32_t owner;
    dump_arm(div, n, false, &owner);
    int64_t t0 = esp_timer_get_time();
    dump_go();
    while (!(REG_READ(DUMP_CTRL) & (1u << 18)) && esp_timer_get_time() - t0 < 300000) {}
    int64_t el = esp_timer_get_time() - t0;
    bool done = (REG_READ(DUMP_CTRL) & (1u << 18)) != 0;
    uint32_t pos = dump_pos16();
    dump_halt(owner);
    unsigned fresh = 0; for (unsigned j = 0; j < n; j++) fresh += w[j] != SENTINEL;
    char h[120]; snprintf(h, sizeof(h), "RATE %u done %d us %" PRId64 " fresh %u pos %" PRIu32 " msps %.3f\n", div, (int)done, el, fresh, pos,
                          el ? (double)n / (double)el : 0.0);
    reply(h);
}

/* B2B <div> <n> <blocks> <work>: back-to-back finite dumps of n words into the
 * lower half. work 0 = none, 1 = energy detector (sum |I|+|Q| per 64), 2 =
 * read every word once (plain load).  Reports mean period, capture and gap. */
static uint32_t *gp_heap;
static void gp_b2b(unsigned div, unsigned n, unsigned blocks, unsigned work) {
    prepare_rx();
    if (work >= 3) {
        if (n > 8192) { reply("ERR n\n"); return; }
        if (!gp_heap) gp_heap = heap_caps_malloc(8192 * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!gp_heap) { reply("ERR mem\n"); return; }
    }
    volatile uint32_t *w = DUMP_WORDS;
    int64_t first = 0, prev = 0, sumcap = 0, maxgap = 0; uint32_t acc = 0;
    for (unsigned b = 0; b < blocks; b++) {
        uint32_t owner;
        dump_arm(div, n, false, &owner);
        int64_t t0 = esp_timer_get_time();
        dump_go();
        while (!(REG_READ(DUMP_CTRL) & (1u << 18)) && esp_timer_get_time() - t0 < 50000) {}
        int64_t t1 = esp_timer_get_time();
        dump_halt(owner);
        if (!b) first = t0; else if (t0 - prev > maxgap) maxgap = t0 - prev;
        sumcap += t1 - t0;
        if (work == 1) {
            for (unsigned j = 0; j < n; j += 64) {
                int32_t e = 0;
                for (unsigned k = 0; k < 64; k++) { uint32_t x = w[j + k]; int32_t I = ((int32_t)(x << 22)) >> 22, Q = ((int32_t)(x << 12)) >> 22; e += (I < 0 ? -I : I) + (Q < 0 ? -Q : Q); }
                acc += e;
            }
        } else if (work == 2) { for (unsigned j = 0; j < n; j++) acc += w[j]; }
        else if (work == 3) { memcpy(gp_heap, DUMP_BUF, n * 4); acc += gp_heap[5]; }
        else if (work == 4) {           /* IQ8 pack, 2 samples per 32-bit store, loads 4 ahead */
            const uint32_t *s = DUMP_BUF; uint32_t *d = gp_heap;
            for (unsigned j = 0; j < n; j += 4, s += 4, d += 2) {
                uint32_t a = s[0], b = s[1], c = s[2], e = s[3];
                d[0] = ((a >> 2) & 0xffu) | ((a >> 4) & 0xff00u) | ((b << 14) & 0xff0000u) | ((b << 12) & 0xff000000u);
                d[1] = ((c >> 2) & 0xffu) | ((c >> 4) & 0xff00u) | ((e << 14) & 0xff0000u) | ((e << 12) & 0xff000000u);
            }
            acc += gp_heap[5];
        }
        prev = esp_timer_get_time();
    }
    int64_t total = prev - first;
    char h[160]; snprintf(h, sizeof(h), "B2B period_us %.1f cap_us %.1f duty %.3f maxgap_us %" PRId64 " acc %" PRIu32 "\n",
                          (double)total / blocks, (double)sumcap / blocks, (double)sumcap / (double)total, maxgap, acc);
    reply(h);
}

static bool gap_probe_command(const char *line) {
    unsigned a, b, c, d; char extra;
    if (sscanf(line, "GDMATEST %u %u %u %c", &a, &b, &c, &extra) == 3 && a <= 5 && b >= 16 && b <= 8000 && c <= 1) { gp_gdmatest(a, b, c); return true; }
    if (sscanf(line, "RATETEST %u %u %c", &a, &b, &extra) == 2 && a <= 7 && b >= 16 && b <= 16380) { gp_ratetest(a, b); return true; }
    if (sscanf(line, "B2B %u %u %u %u %c", &a, &b, &c, &d, &extra) == 4 && a <= 5 && b >= 256 && b <= 16380 && c >= 2 && c <= 2000 && d <= 4) { gp_b2b(a, b, c, d); return true; }
    return false;
}
