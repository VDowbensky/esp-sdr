/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESP32-C5 register-level capture engine (ESPsoup extension, receive only).
 * Included by c5_opt.h; see docs/espsoup-c5.md.
 *
 * Hardware facts this relies on (measured on ESP32-C5 rev 1.0 boards):
 *  - The RF dump writes 32-bit words (I = bits 0-9, Q = bits 10-19, signed)
 *    into SRAM bank 1 (0x40820000-0x4083ffff) while SRAM_OWNER bits 8-11 = 2.
 *    Owner bit 16 selects the 64 KiB half (0 = 0x40820000, 1 = 0x40830000);
 *    the write address is  half + 4 * (position mod 16384).
 *  - DUMP_MODE bits 0-16 are the live write position.  Counts <= 65535 stop by
 *    themselves (CTRL bit 18 = done); larger counts never complete, i.e. the
 *    writer free-runs until CTRL bit 31 is cleared.
 *  - While the dump owns bank 1, neither the CPU nor the GDMA can read any
 *    part of it (stale words).  A finished dump re-arms within ~4 us, so the
 *    continuous engine (c5_cont.h) copies each block out between two dumps.
 *  - Every access to the "source-15 gate" register 0x600a20b4 stalls the bus
 *    for ~0.82 ms; the vendor adctrig() does a read-modify-write on every
 *    snapshot (the fixed ~1.7 ms overhead of the stock path).  The engine
 *    checks the bit once per boot instead.
 *  - phy_set_rf_freq_offset(xtal, MHz, offset) takes the offset in kHz.
 */
#pragma once
#include "esp_heap_caps.h"
#include "driver/temperature_sensor.h"
#include <math.h>

#define DUMP_CTRL 0x600a9004u
#define DUMP_MODE 0x600a9008u
#define DUMP_PACK 0x600a9018u
#define DUMP_WORDS ((volatile uint32_t *)0x40820000u)
#define DUMP_BUF ((uint32_t *)0x40820000u)
#define HALF_WORDS 16384u
#define BIG_MAX 32704u            /* 2 x 16384 minus the worst accepted flip lag */
#define FLIP_MARGIN 64u
#define SENTINEL 0xa5a0055au

static bool opt_stock_engine;     /* ENGINE STOCK: vendor adctrig for <=16380 */
static uint32_t opt_seq;
static int64_t opt_last_t0;
static unsigned opt_last_lag, opt_last_p0, opt_last_p1;

/* ---------------------------------------------------------------- engine */
static inline uint32_t dump_pos16(void) { return REG_READ(DUMP_MODE) & 0xffffu; }

static uint32_t arm_prof[8], arm_gate_writes;
#define ARM_STAMP(k) arm_prof[k] = esp_cpu_get_cycle_count()
static void dump_arm(unsigned div, uint32_t count, bool upper, uint32_t *owner_saved) {
    ARM_STAMP(0);
    *owner_saved = REG_READ(SRAM_OWNER_REG);
    REG_WRITE(DUMP_CTRL, 0);
    ARM_STAMP(1);
    REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE) & ~0x00e00000u) | ((div & 7u) << 21));
    REG_WRITE(0x600a9c04, 0xffffffffu);              /* dump clocks, as adctrig */
    REG_SET_BIT(0x600a0800, 4);
    ARM_STAMP(2);
    REG_WRITE(DUMP_CTRL, count & 0x1ffffu);
    REG_WRITE(DUMP_PACK, (REG_READ(DUMP_PACK) & ~0x00ffffffu) | 24u | (25u << 6) | (26u << 12) | (27u << 18));
    REG_SET_BIT(DUMP_PACK, 1u << 24);
    ARM_STAMP(3);
    REG_WRITE(SRAM_OWNER_REG, (*owner_saved & ~0x10f00u) | 0x200u | (upper ? 0x10000u : 0));
    ARM_STAMP(4);
    __asm__ volatile("fence rw,rw" ::: "memory"); (void)REG_READ(SRAM_OWNER_REG);
    ARM_STAMP(5);
    /* Source-15 gate at 0x600a20b4. Every access to this register stalls the
     * bus for ~0.82 ms (read) / 1.64 ms (read-modify-write, as in the vendor
     * adctrig) - this is the fixed ~1.7 ms per-snapshot overhead of the stock
     * firmware. The bit was found clear after boot and stays clear, so check
     * it once per boot only. */
    static bool gate_checked;
    if (!gate_checked) { if (REG_READ(0x600a20b4) & 1u) { REG_CLR_BIT(0x600a20b4, 1u); arm_gate_writes++; } gate_checked = true; }
    ARM_STAMP(6);
    REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE) & ~0x001e0000u) | (15u << 17));
    ARM_STAMP(7);
}
static inline void dump_go(void) {
    REG_SET_BIT(DUMP_CTRL, 1u << 31);
    REG_SET_BIT(DUMP_CTRL, 1u << 19); REG_CLR_BIT(DUMP_CTRL, 1u << 19);
}
static void dump_halt(uint32_t owner_saved) {
    REG_CLR_BIT(DUMP_CTRL, 1u << 31);
    REG_WRITE(DUMP_CTRL, 0);
    REG_WRITE(SRAM_OWNER_REG, owner_saved);
    __asm__ volatile("fence rw,rw" ::: "memory"); (void)REG_READ(SRAM_OWNER_REG);
}
/* Spin (interrupts off) until the 16-bit position passes target, then toggle the
 * half.  Returns positions read before (p0) and after (p1) the owner write. */
static inline void IRAM_ATTR dump_flip_at(uint32_t target16, uint32_t *p0, uint32_t *p1) {
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    uint32_t o = REG_READ(SRAM_OWNER_REG) ^ 0x10000u, p;
    unsigned guard = 400000;       /* ~20 ms at 4 MS/s; no timer call inside the spin */
    do { p = REG_READ(DUMP_MODE) & 0xffffu; } while (((p - target16) & 0x8000u) && --guard);
    REG_WRITE(SRAM_OWNER_REG, o);
    *p1 = REG_READ(DUMP_MODE) & 0xffffu;
    *p0 = p;
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
}

/* Contiguous snapshot of n <= BIG_MAX samples into DUMP_BUF[0..n).
 * n <= 16380: one finite dump into the lower half (stock-equivalent).
 * n  > 16380: two halves; the flip lag L is measured exactly from sentinels
 * left in the upper half and the first L samples are dropped, then the data
 * is re-ordered in place so the caller sees one contiguous block. */
static const char *fast_capture(unsigned n, unsigned div, uint32_t *elapsed) {
    volatile uint32_t *w = DUMP_WORDS;
    bool big = n > HALF_WORDS - 4;
    if (big) { for (unsigned j = 0; j < 2 * HALF_WORDS; j++) w[j] = SENTINEL; }
    else {
        for (unsigned j = 0; j < n; j++) w[j] = SENTINEL;
        for (unsigned j = 0; j < 4; j++) w[n + j] = 0x5a5aa5a5u ^ j;
    }
    unsigned analog_saved[2]; rx_analog_apply(analog_saved);
    uint32_t owner;
    dump_arm(div, big ? 2 * HALF_WORDS : n, false, &owner);
    int64_t t0 = esp_timer_get_time();
    dump_go();
    uint32_t p0 = 0, p1 = 0;
    if (big) dump_flip_at(HALF_WORDS, &p0, &p1);
    int64_t lim = t0 + 20000;
    while (!(REG_READ(DUMP_CTRL) & (1u << 18)) && esp_timer_get_time() < lim) {}
    bool done = (REG_READ(DUMP_CTRL) & (1u << 18)) != 0;
    *elapsed = (uint32_t)(esp_timer_get_time() - t0);
    dump_halt(owner);
    rx_analog_restore(analog_saved);
    opt_last_t0 = t0; opt_last_p0 = p0; opt_last_p1 = p1; opt_last_lag = 0;
    if (!done) return "ERR capture_timeout\n";
    if (!big) {
        for (unsigned j = 0; j < n; j++) if (w[j] == SENTINEL) return "ERR capture_timeout\n";
        for (unsigned j = 0; j < 4; j++) if (w[n + j] != (0x5a5aa5a5u ^ j)) return "ERR capture_overrun\n";
        return NULL;
    }
    unsigned L = 0;
    while (L < FLIP_MARGIN + 1 && w[HALF_WORDS + L] == SENTINEL) L++;
    opt_last_lag = L;
    if (L > FLIP_MARGIN) return "ERR capture_flip\n";
    uint32_t tmp[FLIP_MARGIN];
    for (unsigned j = 0; j < L; j++) tmp[j] = w[j];
    memmove(DUMP_BUF, DUMP_BUF + L, (HALF_WORDS - L) * 4);
    for (unsigned j = 0; j < L; j++) DUMP_BUF[HALF_WORDS - L + j] = tmp[j];
    memmove(DUMP_BUF + HALF_WORDS, DUMP_BUF + HALF_WORDS + L, (n - HALF_WORDS) * 4);
    for (unsigned j = 0; j < n; j++) if (DUMP_BUF[j] == SENTINEL) return "ERR capture_timeout\n";
    return NULL;
}

/* ------------------------------------------------------------- packing */
static void opt_pack(uint32_t *b, unsigned n, unsigned format) {
    uint8_t *p = (uint8_t *)b;
    if (format == 16) {
        for (unsigned j = 0; j < n; j++) { uint32_t x = b[j]; p[2 * j] = (x >> 2) & 255; p[2 * j + 1] = (x >> 12) & 255; }
    } else if (format == 20) {
        for (unsigned j = 0; j < n; j += 2, p += 5) {
            uint32_t a = b[j] & 0xfffffu, c = j + 1 < n ? b[j + 1] & 0xfffffu : 0;
            p[0] = a; p[1] = a >> 8; p[2] = (a >> 16) | (c << 4);
            if (j + 1 < n) { p[3] = c >> 4; p[4] = c >> 12; }
        }
    }
}
static size_t opt_wire(unsigned n, unsigned format) {
    return format == 16 ? n * 2 : format == 20 ? (n * 20u + 7u) / 8u : n * 4;
}
/* Header "DATA n crc us" (stock) or "DATAT n crc us t0_us seq lag" (stamped). */
static bool opt_send_frame(const void *payload, size_t bytes, unsigned n, uint32_t elapsed, bool stamped) {
    uint32_t crc = esp_rom_crc32_le(0, payload, bytes);
    char h[128];
    if (stamped)
        snprintf(h, sizeof(h), "DATAT %u %08" PRIx32 " %" PRIu32 " %" PRId64 " %" PRIu32 " %u\n", n, crc, elapsed,
                 opt_last_t0, ++opt_seq, opt_last_lag);
    else snprintf(h, sizeof(h), "DATA %u %08" PRIx32 " %" PRIu32 "\n", n, crc, elapsed);
    return send_bytes(h, strlen(h)) && send_bytes(payload, bytes);
}
static bool opt_capture(unsigned n, unsigned div, unsigned format, bool stamped) {
    prepare_rx();
    uint32_t elapsed;
    const char *err = fast_capture(n, div, &elapsed);
    if (err) { reply(err); return false; }
    opt_pack(DUMP_BUF, n, format);
    return opt_send_frame(DUMP_BUF, opt_wire(n, format), n, elapsed, stamped);
}

/* ---------------------------------------------------------- temperature */
static temperature_sensor_handle_t opt_tsens;
static bool opt_temp(float *c) {
    if (!opt_tsens) {
        temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
        if (temperature_sensor_install(&cfg, &opt_tsens) != ESP_OK) { opt_tsens = NULL; return false; }
        if (temperature_sensor_enable(opt_tsens) != ESP_OK) return false;
    }
    return temperature_sensor_get_celsius(opt_tsens, c) == ESP_OK;
}

/* Snapshot source for the shared on-chip spectrum (spectrum.c, upstream SPEC
 * protocol): the same capture as acquire_iq() without the ~1.7 ms vendor stall. */
static bool opt_acquire(unsigned n, unsigned div, const uint32_t **words, unsigned *elapsed_us) {
    prepare_rx();
    uint32_t elapsed;
    const char *err = fast_capture(n, div, &elapsed);
    if (err) { reply(err); return false; }
    *words = DUMP_BUF; *elapsed_us = elapsed;
    return true;
}

static char hm_buf[400]; static int hm_len;
static bool hm_walk(walker_heap_into_t hi, walker_block_info_t b, void *u) {
    (void)hi; (void)u;
    if (!b.used && b.size >= 1024 && hm_len < (int)sizeof(hm_buf) - 24)
        hm_len += snprintf(hm_buf + hm_len, sizeof(hm_buf) - hm_len, " %08x:%u", (unsigned)(uintptr_t)b.ptr, (unsigned)b.size);
    return true;
}

/* ------------------------------------------------------------ commands */
static bool engine_command(const char *line) {
    unsigned n, rate, khz; char extra;
    if (!strcmp(line, "ENGINE STOCK")) { opt_stock_engine = true; reply("OK\n"); return true; }
    if (!strcmp(line, "ENGINE FAST")) { opt_stock_engine = false; reply("OK\n"); return true; }
    if (!strcmp(line, "ENGINE?")) { reply(opt_stock_engine ? "ENGINE STOCK\n" : "ENGINE FAST\n"); return true; }
    if (!strcmp(line, "TEMP?")) {
        float c; char h[48];
        if (!opt_temp(&c)) { reply("ERR temp\n"); return true; }
        snprintf(h, sizeof(h), "TEMP %.1f\n", (double)c); reply(h); return true;
    }
    if (!strcmp(line, "TIME?")) {
        char h[48]; snprintf(h, sizeof(h), "TIME %" PRId64 "\n", esp_timer_get_time()); reply(h); return true;
    }
    if (!strcmp(line, "HEAP?")) {
        char h[80]; snprintf(h, sizeof(h), "HEAP %u %u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); reply(h); return true;
    }
    if (!strcmp(line, "HEAPMAP?")) {   /* free heap blocks >= 1 KiB: addr:size */
        hm_len = snprintf(hm_buf, sizeof(hm_buf), "HEAPMAP");
        heap_caps_walk(MALLOC_CAP_8BIT, hm_walk, NULL);
        snprintf(hm_buf + hm_len, sizeof(hm_buf) - hm_len, "\n"); reply(hm_buf); return true;
    }
    if (sscanf(line, "IDLETEST %u %c", &n, &extra) == 1 && n <= 5) {
        /* diagnostic: read the idle half while the writer fills the other one, then again after release */
        prepare_rx();
        static uint32_t during[8], after[8]; uint32_t owner, p0, p1; char h[200];
        for (unsigned j = 0; j < 2 * HALF_WORDS; j++) DUMP_WORDS[j] = SENTINEL;
        dump_arm(n, 0x1ffffu, false, &owner); dump_go();
        dump_flip_at(HALF_WORDS, &p0, &p1);
        for (unsigned j = 0; j < 8; j++) during[j] = DUMP_WORDS[1000 + j * 997];
        dump_halt(owner);
        for (unsigned j = 0; j < 8; j++) after[j] = DUMP_WORDS[1000 + j * 997];
        int o = snprintf(h, sizeof(h), "IDLE");
        for (unsigned j = 0; j < 8; j++) o += snprintf(h + o, sizeof(h) - o, " %08" PRIx32 "/%08" PRIx32, during[j], after[j]);
        snprintf(h + o, sizeof(h) - o, "\n"); reply(h); return true;
    }
    if (!strcmp(line, "ARMPROF?")) {
        char h[128]; int o = snprintf(h, sizeof(h), "ARMPROF gate_writes %" PRIu32, arm_gate_writes);
        for (unsigned j = 1; j < 8; j++) o += snprintf(h + o, sizeof(h) - o, " %" PRIu32, arm_prof[j] - arm_prof[j - 1]);
        snprintf(h + o, sizeof(h) - o, "\n"); reply(h); return true;
    }
    if (!strcmp(line, "LASTCAP?")) {
        char h[96]; snprintf(h, sizeof(h), "LASTCAP %" PRId64 " %u %u %u\n", opt_last_t0, opt_last_lag, opt_last_p0, opt_last_p1);
        reply(h); return true;
    }
    /* FREQK <kHz>: PLL tuning in 1 kHz steps (integer MHz + kHz offset). */
    if (sscanf(line, "FREQK %u %c", &khz, &extra) == 1) {
        unsigned mhz = (khz + 500) / 1000; int o = (int)khz - (int)mhz * 1000;
        if (!frequency_valid(mhz)) { reply("ERR command\n"); return true; }
        frequency_mhz = mhz; c5_khz_offset = o; rx_ready = false; prepare_rx();
        reply("OK\n"); return true;
    }
    if (!strcmp(line, "FREQK?")) {
        char h[48]; snprintf(h, sizeof(h), "FREQK %d\n", (int)frequency_mhz * 1000 + c5_khz_offset); reply(h); return true;
    }
    /* CAPT16/CAPT20 <n> <rate>: like CAP16/CAP20 (up to BIG_MAX) with a
     * "DATAT n crc us t0 seq lag" header (t0 = us since boot at dump start). */
    if (sscanf(line, "CAPT16 %u %u %c", &n, &rate, &extra) == 2 || sscanf(line, "CAPT20 %u %u %c", &n, &rate, &extra) == 2) {
        if (n < 256 || n > BIG_MAX || rate > 5) { reply("ERR command\n"); return true; }
        opt_capture(n, rate, line[4] == '1' ? 16 : 20, true); return true;
    }
    return false;
}
