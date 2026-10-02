/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESP32-C5 gap-free I/Q path (ESPsoup extension, receive only):
 * modem diagnostic bus -> GPIO matrix -> PARLIO RX -> GDMA ring in SRAM bank 1.
 *
 * Attribution: the idea of routing the modem diagnostic (MODEM_DIAG) signals to
 * pads through the GPIO matrix and reading them back with PARLIO RX, and of
 * muting the Wi-Fi promiscuous filter during a run, comes from the C5VRX project
 * by konradit (GPL-3.0-only).  This file is an independent implementation; no
 * C5VRX code was copied.  Signal indices come from soc/gpio_sig_map.h
 * (MODEM_DIAG0_IDX), the dump registers from c5_engine.h; bit assignment, clock
 * and levels were determined by our own measurements.  See NOTICE.
 *
 * Measured on ESP32-C5 rev 1.0 boards:
 *  - MODEM_DIAG[0:19] carries the dump word: I = DIAG[0:9], Q = DIAG[10:19] (10-bit two's complement, device
 *    orientation like the dump; the host negates Q).  DIAG[20:31] are nearly static status bits.
 *  - With the pad output AND input enabled, PARLIO RX samples the pad level internally: no wires.
 *  - PARLIO RX delivers at most 40 MB/s (80/60 MHz requested -> 40.000 MB/s measured): 40 MS/s of 8 bits.
 *    PARLIO RX is at most 8 bits wide -> a 4-bit window of I and Q: byte = I[lsb+3:lsb] | Q[lsb+3:lsb] << 4
 *    (wraps when the signal exceeds the window, hence the level check / PLSB AUTO).
 *  - Infinite transaction with a soft delimiter (EOF every ring length) into a ring in SRAM bank 1 (the dump
 *    window, free while the CPU owns it): 62 s, 2.48e9 bytes, ~50000 ring wraps, bytes vs. esp_timer +0.006 ppm,
 *    i.e. no stall at the wraps; 0 overruns.
 *
 * CAUTION - side effect: during a PARLIO run the 8 data pads (default GPIO 0 1 3 4 5 7 10 23, see pc_pins,
 * change with PPINS) are driven as outputs and toggle at up to 40 MHz (lowest drive strength).  Do not use the
 * PARLIO path (VFMSRC 0, STRSRC 0, no PCAP/PRUN/PDIAG) on a board that has anything connected to those pins.
 */
#pragma once
#include "driver/parlio_rx.h"
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "soc/io_mux_reg.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"

/* ------------------------------------------------------------ routing */
/* Default pads: header pads on the ESP32-C5-DevKitC-1 and the XIAO ESP32-C5 (not USB 13/14, flash/PSRAM 15-22,
 * UART0 11/12, RGB LED 27, BOOT 28).  Measured loopback consistency (same DIAG signal on all lines):
 * GPIO 0-10, 23, 24 agree 100 %, GPIO 25 only 98.2 %, GPIO 26 94.8 % -> not used.  PPINS changes them. */
static int8_t pc_pins[8] = {0, 1, 3, 4, 5, 7, 10, 23};
static uint8_t pc_map[8] = {6, 7, 8, 9, 16, 17, 18, 19}; /* PMAP: data line k <- MODEM_DIAG[pc_map[k]] (default I[9:6], Q[9:6]) */
static unsigned pc_drive = 0;                             /* PDRIVE: GPIO_DRIVE_CAP_0..3 */
static int pc_lsb = -1;                                   /* PLSB: window I[lsb+3:lsb]; -1 = AUTO (level check before a run) */
static unsigned pc_edge;                                  /* PEDGE: 0 sample on the rising PARLIO clock edge, 1 falling */
static unsigned pc_last_lsb = 6, pc_last_peak;            /* last AUTO result (PSTAT?) */

static void pc_map_window(uint8_t *m, unsigned lsb) {
    for (unsigned k = 0; k < 4; k++) { m[k] = (uint8_t)(lsb + k); m[4 + k] = (uint8_t)(10 + lsb + k); }
}
static void pc_route(const uint8_t *map) {
    for (unsigned k = 0; k < 8; k++) {
        int p = pc_pins[k];
        if (p < 0) continue;
        gpio_func_sel(p, PIN_FUNC_GPIO);
        gpio_set_pull_mode(p, GPIO_FLOATING);
        gpio_set_drive_capability(p, (gpio_drive_cap_t)pc_drive);
        gpio_input_enable(p);
        esp_rom_gpio_connect_out_signal(p, MODEM_DIAG0_IDX + map[k], false, false);   /* also sets the output enable */
    }
}
static void pc_unroute(void) {
    for (unsigned k = 0; k < 8; k++) if (pc_pins[k] >= 0) gpio_reset_pin(pc_pins[k]);
}

/* ------------------------------------------------- modem bus kept running */
/* PMODEM flags: 1 = dump engine free-running (count > 65535 never completes, as in fast_capture's long counts),
 * 2 = with SRAM ownership given to the modem (else the CPU keeps bank 1 and the ring may live there),
 * 4 = selector 3 + dump-first bit instead of the software trigger.  0 = only divider/clock registers.
 * The bus toggles in every mode (PDIAG); 1 is the default (measured with it). */
static unsigned pc_modem_flags = 1;
static bool pc_modem_on; static uint32_t pc_owner_saved; static unsigned pc_analog[2];
static void pc_modem_start(unsigned div) {
    if (pc_modem_on) return;
    prepare_rx(); rx_analog_apply(pc_analog);
    pc_owner_saved = REG_READ(SRAM_OWNER_REG);
    if (pc_modem_flags & 1) {
        uint32_t owner;
        dump_arm(div, 0x1ffffu, false, &owner);                 /* div, clocks, packing, owner = modem */
        if (!(pc_modem_flags & 2)) {                            /* give bank 1 back to the CPU */
            REG_WRITE(SRAM_OWNER_REG, pc_owner_saved);
            __asm__ volatile("fence rw,rw" ::: "memory"); (void)REG_READ(SRAM_OWNER_REG);
        }
        if (pc_modem_flags & 4) {
            REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE) & ~0x001e0000u) | (3u << 17));
            REG_WRITE(DUMP_CTRL, (REG_READ(DUMP_CTRL) & 0x1ffffu) | (1u << 17));
            REG_SET_BIT(DUMP_CTRL, 1u << 31);
        } else dump_go();
    } else {
        REG_WRITE(DUMP_MODE, (REG_READ(DUMP_MODE) & ~0x00e00000u) | ((div & 7u) << 21));
        REG_WRITE(0x600a9c04, 0xffffffffu); REG_SET_BIT(0x600a0800, 4);
    }
    pc_modem_on = true;
}
static void pc_modem_stop(void) {
    if (!pc_modem_on) return;
    if (pc_modem_flags & 1) dump_halt(pc_owner_saved);
    rx_analog_restore(pc_analog);
    pc_modem_on = false;
}

/* --------------------------------------------- PDIAG: bus activity scan */
/* Route the 32 diagnostic signals 8 at a time and count level changes of each pad in n CPU reads of GPIO_IN
 * (CPU polling: activity / bit statistics only, not the sample rate).  Then the share of ones per signal. */
static void pc_diag_scan(unsigned n, unsigned div) {
    char h[400]; int o = snprintf(h, sizeof(h), "PDIAG");
    pc_modem_start(div);
    uint32_t ones[32];
    for (unsigned g = 0; g < 4; g++) {
        uint8_t m[8]; for (unsigned k = 0; k < 8; k++) m[k] = (uint8_t)(g * 8 + k);
        pc_route(m);
        esp_rom_delay_us(20);
        uint32_t cnt[8] = {0}, on[8] = {0}, prev = REG_READ(GPIO_IN_REG);
        for (unsigned j = 0; j < n; j++) {
            uint32_t x = REG_READ(GPIO_IN_REG), d = x ^ prev; prev = x;
            for (unsigned k = 0; k < 8; k++) { cnt[k] += (d >> pc_pins[k]) & 1u; on[k] += (x >> pc_pins[k]) & 1u; }
        }
        for (unsigned k = 0; k < 8; k++) { o += snprintf(h + o, sizeof(h) - o, " %" PRIu32, cnt[k]); ones[g * 8 + k] = on[k]; }
    }
    o += snprintf(h + o, sizeof(h) - o, " |");
    for (unsigned k = 0; k < 32; k++) o += snprintf(h + o, sizeof(h) - o, " %" PRIu32, ones[k] * 100 / (n ? n : 1));
    pc_unroute();
    pc_modem_stop();
    snprintf(h + o, sizeof(h) - o, "\n");
    reply(h);
}

/* --------------------------------------------------- PARLIO RX unit */
typedef struct {
    parlio_rx_unit_handle_t unit; parlio_rx_delimiter_handle_t delim;
    volatile uint32_t prod;                /* bytes written by the GDMA (free-running, 32-bit: atomic reads) */
    volatile uint32_t nodes, eofs;         /* ISR counters */
    volatile int64_t t_first, t_last;      /* esp_timer at the first / last node completion */
    volatile uint32_t prod_first, prod_last;
    uint32_t clk;
} pc_rx_t;
static pc_rx_t pc_rx;

static bool IRAM_ATTR pc_on_partial(parlio_rx_unit_handle_t u, const parlio_rx_event_data_t *e, void *arg) {
    pc_rx_t *r = (pc_rx_t *)arg; (void)u;
    int64_t t = esp_timer_get_time();
    uint32_t p = r->prod + e->recv_bytes;
    r->prod = p; r->nodes++;
    if (!r->t_first) { r->t_first = t; r->prod_first = p; }
    r->t_last = t; r->prod_last = p;
    return false;
}
static bool IRAM_ATTR pc_on_done(parlio_rx_unit_handle_t u, const parlio_rx_event_data_t *e, void *arg) {
    (void)u; (void)e; ((pc_rx_t *)arg)->eofs++; return false;
}

/* Create + enable the unit; ring = DMA target of size bytes (<= 65535: the soft delimiter's EOF length). */
static esp_err_t pc_open(uint32_t clk, unsigned size) {
    pc_rx_t *r = &pc_rx;
    memset(r, 0, sizeof(*r));
    r->clk = clk;
    parlio_rx_unit_config_t cfg = {
        .trans_queue_depth = 2, .max_recv_size = size, .dma_burst_size = 32, .data_width = 8,
        .clk_src = PARLIO_CLK_SRC_PLL_F240M, .exp_clk_freq_hz = clk,
        .clk_in_gpio_num = -1, .clk_out_gpio_num = -1, .valid_gpio_num = -1,
    };
    for (unsigned k = 0; k < 8; k++) cfg.data_gpio_nums[k] = pc_pins[k];
    esp_err_t e = parlio_new_rx_unit(&cfg, &r->unit);
    if (e) return e;
    parlio_rx_soft_delimiter_config_t dc = {
        .sample_edge = pc_edge ? PARLIO_SAMPLE_EDGE_NEG : PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB, .eof_data_len = size, .timeout_ticks = 0,
    };
    if ((e = parlio_new_rx_soft_delimiter(&dc, &r->delim))) { parlio_del_rx_unit(r->unit); r->unit = NULL; return e; }
    parlio_rx_event_callbacks_t cbs = { .on_partial_receive = pc_on_partial, .on_receive_done = pc_on_done };
    parlio_rx_unit_register_event_callbacks(r->unit, &cbs, r);
    if ((e = parlio_rx_unit_enable(r->unit, true))) { parlio_del_rx_delimiter(r->delim); parlio_del_rx_unit(r->unit); r->unit = NULL; return e; }
    return ESP_OK;
}
static esp_err_t pc_receive(uint8_t *buf, unsigned size, bool endless) {
    parlio_receive_config_t rc = { .delimiter = pc_rx.delim, .flags.partial_rx_en = endless };
    parlio_rx_soft_delimiter_start_stop(pc_rx.unit, pc_rx.delim, true);
    return parlio_rx_unit_receive(pc_rx.unit, buf, size, &rc);
}
static void pc_close(void) {
    pc_rx_t *r = &pc_rx;
    if (!r->unit) return;
    parlio_rx_soft_delimiter_start_stop(r->unit, r->delim, false);
    parlio_rx_unit_disable(r->unit);
    parlio_del_rx_delimiter(r->delim);
    parlio_del_rx_unit(r->unit);
    r->unit = NULL;
}

/* Bank 1 (the dump window) is free while the dump does not own it: ring / snapshot buffer there (no heap). */
#define PC_BANK1 ((uint8_t *)0x40820000u)
#define PC_BANK1_SIZE 131072u
static bool pc_use_bank1(void) { return !(pc_modem_flags & 2); }

/* One finite snapshot of n bytes with the given line map into buf (bank 1 or heap). */
static esp_err_t pc_snap(uint8_t *buf, unsigned n, const uint8_t *map, unsigned div, int64_t *t0) {
    memset(buf, 0xee, n);
    pc_modem_start(div);
    pc_route(map);
    esp_err_t e = pc_open(40000000, n);
    if (!e) {
        e = pc_receive(buf, n, false);
        if (!e) e = parlio_rx_unit_wait_all_done(pc_rx.unit, 200);
    }
    if (t0) *t0 = pc_rx.t_first;
    pc_close();
    pc_unroute();
    pc_modem_stop();
    return e;
}

/* ------------------------------------------- level check (PLSB AUTO) */
/* Two 16 KB snapshots of I[9:2] and Q[9:2] (8 lines each); peak = 99.9 % point of |x| (incl. DC, LSB);
 * window = the lowest lsb (2..6) with 7.5 * 2^lsb >= 1.15 * peak, so the 4-bit codes neither wrap nor waste range. */
static bool pc_level_robust;      /* set by the VFM run: window from the median amplitude (see below) */
static unsigned pc_level(unsigned *peak_lsb) {
    uint8_t *buf = pc_use_bank1() ? PC_BANK1 : heap_caps_malloc(16384, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf) { *peak_lsb = 0; return 6; }
    uint32_t h[129]; memset(h, 0, sizeof(h));
    for (unsigned c = 0; c < 2; c++) {
        uint8_t m[8]; for (unsigned k = 0; k < 8; k++) m[k] = (uint8_t)(c * 10 + 2 + k);
        if (pc_snap(buf, 16384, m, 0, NULL)) continue;
        for (unsigned j = 0; j < 16384; j++) { int v = (int8_t)buf[j]; h[v < 0 ? -v : v]++; }
    }
    if (!pc_use_bank1()) free(buf);
    uint32_t tot = 0, acc = 0; for (unsigned k = 0; k <= 128; k++) tot += h[k];
    unsigned pk = 128, med = 0; bool have_med = false;
    for (unsigned k = 0; k <= 128; k++) {
        acc += h[k];
        if (!have_med && acc * 2ull >= tot) { med = k; have_med = true; }
        if (acc * 1000ull >= tot * 999ull) { pk = k; break; }
    }
    unsigned peak = pk * 4 + 2;                                  /* LSB of the 10-bit word */
    /* VFM (pc_level_robust): the FM carrier is always there, bursts (Wi-Fi at 2.4 GHz) are not - a burst in the
     * snapshot made the 99.9 % point pick lsb 6 for a 14-90 LSB carrier (137 relocks in 60 s).  Constant envelope:
     * median |x| of one component ~ 0.7 A  ->  peak estimate 2.2 x median (incl. noise margin). */
    if (pc_level_robust) { unsigned pm = (med * 4 + 2) * 22 / 10; if (pm < peak) peak = pm; }
    unsigned lsb = 2;
    while (lsb < 6 && (15u << lsb) < peak * 23u / 10u) lsb++;    /* 7.5 * 2^lsb >= 1.15 * peak */
    *peak_lsb = peak;
    return lsb;
}

/* ------------------------------------------------- PCAP: one snapshot */
/* PCAP <n> [<div> [<where>]] -> PDATA n crc t0_us lsb nodes + n bytes (current PMAP; PLSB n sets the map, AUTO measures) */
static void pc_capture(unsigned n, unsigned div, unsigned where) {
    bool heap = where == 1 || !pc_use_bank1();
    uint8_t *buf = heap ? heap_caps_aligned_alloc(64, n, MALLOC_CAP_DMA | MALLOC_CAP_8BIT) : PC_BANK1;
    if (!buf) { reply("ERR pcap_memory\n"); return; }
    int64_t t0 = 0;
    esp_err_t e = pc_snap(buf, n, pc_map, div, &t0);
    if (e) { char m[48]; snprintf(m, sizeof(m), "ERR pcap %d\n", (int)e); reply(m); if (heap) free(buf); return; }
    uint32_t crc = esp_rom_crc32_le(0, buf, n);
    char h[96]; snprintf(h, sizeof(h), "PDATA %u %08" PRIx32 " %" PRId64 " %d %" PRIu32 "\n", n, crc, t0, pc_lsb, pc_rx.nodes);
    send_bytes(h, strlen(h)); send_bytes(buf, n);
    if (heap) free(buf);
}

/* --------------------------------------------- continuous ring engine */
/* The ring (PE_RING bytes, 6 blocks of PE_BLK) is written by the GDMA without pause.  pe_next() hands out the
 * next complete block in place (no copy); if the consumer fell more than the ring minus one block behind, it
 * jumps to the newest complete block and reports the skipped bytes as gap (exact, counted in bytes = samples).
 * pe_done() checks afterwards that the producer did not overwrite the block while it was being processed. */
#define PE_RING 57344u
#define PE_BLK 8192u
typedef struct {
    uint8_t *ring; unsigned lsb, peak;
    uint32_t cons;                  /* bytes consumed or skipped (32-bit, same wrap as pc_rx.prod) */
    uint64_t cons64; unsigned ri;   /* 64-bit total, ring offset of the next block */
    uint32_t blocks, overruns, late; uint64_t lost; uint32_t maxlag;
    uint64_t work_cyc; int64_t t_start, t_end;
} pe_t;
static pe_t pe_last; static bool pe_have;

static unsigned pc_dccal = 1;     /* PDC: hardware DC correction (dc_cal.h) before every PARLIO run (MANUAL gain only) */
/* During a run the Wi-Fi MAC must not hand received frames to the driver task (it preempts the main task for
 * long enough to overrun the ring on a busy 2.4 GHz channel): promiscuous filter 0 (idea: C5VRX), restored after. */
static wifi_promiscuous_filter_t pe_filter_saved; static bool pe_filter_have;
static void pe_quiet_wifi(bool on) {
    if (on) {
        pe_filter_have = esp_wifi_get_promiscuous_filter(&pe_filter_saved) == ESP_OK;
        wifi_promiscuous_filter_t f = { .filter_mask = 0 };
        (void)esp_wifi_set_promiscuous_filter(&f);
    } else if (pe_filter_have) { (void)esp_wifi_set_promiscuous_filter(&pe_filter_saved); pe_filter_have = false; }
}
static esp_err_t pe_start(pe_t *e) {
    memset(e, 0, sizeof(*e));
    pe_quiet_wifi(true);
    if (pc_dccal) dc_calibrate();   /* DC eats the 4-bit window: measured 30-43 ADC8 counts at 5 GHz / gain 60 -> < 0.5 */
    e->lsb = pc_lsb >= 0 ? (unsigned)pc_lsb : pc_level(&e->peak);
    if (pc_lsb < 0) { pc_last_lsb = e->lsb; pc_last_peak = e->peak; }
    e->ring = pc_use_bank1() ? PC_BANK1 : heap_caps_aligned_alloc(64, PE_RING, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!e->ring) return ESP_ERR_NO_MEM;
    uint8_t m[8]; pc_map_window(m, e->lsb);
    pc_modem_start(0);
    pc_route(m);
    esp_err_t r = pc_open(40000000, PE_RING);
    if (!r) r = pc_receive(e->ring, PE_RING, true);
    if (r) { pc_close(); pc_unroute(); pc_modem_stop(); pe_quiet_wifi(false); if (!pc_use_bank1()) free(e->ring); e->ring = NULL; }
    e->t_start = esp_timer_get_time();
    return r;
}
/* Next block or NULL (timeout).  *gap = bytes skipped before it, *t16 = time of its first byte in 1/16 us. */
static const uint8_t *pe_next(pe_t *e, uint32_t *gap, int64_t *t16) {
    int64_t lim = esp_timer_get_time() + 100000;
    while ((uint32_t)(pc_rx.prod - e->cons) < PE_BLK) { ring_pump(); if (esp_timer_get_time() > lim) return NULL; }
    uint32_t lag = pc_rx.prod - e->cons; *gap = 0;
    if (lag > e->maxlag) e->maxlag = lag;
    if (lag > PE_RING - PE_BLK) {                       /* lapped: skip to the newest complete block */
        uint32_t skip = (lag / PE_BLK - 1) * PE_BLK;
        e->cons += skip; e->cons64 += skip; e->ri = (e->ri + skip) % PE_RING;
        *gap = skip; e->lost += skip; e->overruns++;
    }
    const uint8_t *x = e->ring + e->ri;
    *t16 = pc_rx.t_first * 16 + ((int64_t)e->cons64 - (int64_t)pc_rx.prod_first) * 2 / 5;   /* 40 bytes per us */
    e->cons += PE_BLK; e->cons64 += PE_BLK; e->ri = (e->ri + PE_BLK) % PE_RING; e->blocks++;
    return x;
}
/* After processing the block from pe_next: was it overwritten meanwhile? */
static void pe_done(pe_t *e) { if ((uint32_t)(pc_rx.prod - (e->cons - PE_BLK)) > PE_RING) e->late++; }
static void pe_stop(pe_t *e) {
    e->t_end = esp_timer_get_time();
    pc_close(); pc_unroute(); pc_modem_stop();
    pe_quiet_wifi(false);
    if (e->ring && !pc_use_bank1()) free(e->ring);
    e->ring = NULL;
    pe_last = *e; pe_have = true;
}

/* Any line from the host stops a run (STOP). */
static bool pc_poll_stop(void) { char l[48]; return burst_serial_poll_run_line(l, sizeof(l)); }

/* Boxcar 16 -> one dump-format word per 16 bytes (2.5 MS/s): I, Q = 2 * sum of the 16 codes as (2k + 1), i.e. +-480,
 * which equals the 10-bit LSB scale for the window lsb 6.  SWAR on four 32-bit loads: xor 0x8 turns the signed
 * nibble k into k + 8 (0..15); the byte lanes are summed in parallel (each <= 60) and folded with one multiply
 * (total <= 240 < 256).  ~37 instructions per 16 bytes (measured with STREAM, STREAMSTAT "capture"). */
static void pc_box16(const uint8_t *x, unsigned nw, uint32_t *w) {
    const uint32_t *p = (const uint32_t *)x; const uint32_t M = 0x0f0f0f0fu, X = 0x88888888u;
    for (unsigned j = 0; j < nw; j++, p += 4) {
        uint32_t a = p[0] ^ X, b = p[1] ^ X, c = p[2] ^ X, d = p[3] ^ X;
        uint32_t lo = (a & M) + (b & M) + (c & M) + (d & M);
        uint32_t hi = ((a >> 4) & M) + ((b >> 4) & M) + ((c >> 4) & M) + ((d >> 4) & M);
        int32_t si = (int32_t)((lo * 0x01010101u) >> 24), sq = (int32_t)((hi * 0x01010101u) >> 24);
        w[j] = ((uint32_t)(4 * si - 480) & 1023u) | (((uint32_t)(4 * sq - 480) & 1023u) << 10);
    }
}

/* --------------------------------------- PRUN: endless ring, rate check */
/* PRUN <seconds> [<ring>]: infinite transaction into the ring; the CPU follows the producer and touches every word
 * (running XOR, like a consumer).  Once a second "PST t_us bytes nodes eofs overruns ppm xor"; END compares the
 * bytes received between the first and last node completion with 40 MHz x elapsed (a stall of k clocks at every
 * ring wrap would show as -k/ring).  The rest of bank 1 is filled with a pattern before and checked after
 * (does the free-running dump engine write bank 1 while the CPU owns it?). */
static void pc_run(unsigned seconds, unsigned ring) {
    if (ring < 4096 || ring > 65532 || (ring & 3)) { reply("ERR prun_ring\n"); return; }
    bool b1 = pc_use_bank1();
    uint8_t *buf = b1 ? PC_BANK1 : heap_caps_aligned_alloc(64, ring, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf) { reply("ERR prun_memory\n"); return; }
    uint32_t *guard = (uint32_t *)(PC_BANK1 + ((ring + 63) & ~63u)); unsigned ng = b1 ? (PC_BANK1_SIZE - ((ring + 63) & ~63u)) / 4 : 0;
    for (unsigned j = 0; j < ng; j++) guard[j] = 0x5a5aa5a5u ^ j;
    uint8_t m[8]; if (pc_lsb >= 0) pc_map_window(m, (unsigned)pc_lsb); else memcpy(m, pc_map, 8);
    pc_modem_start(0);
    pc_route(m);
    esp_err_t e = pc_open(40000000, ring);
    if (!e) e = pc_receive(buf, ring, true);
    if (e) { pc_close(); pc_unroute(); pc_modem_stop(); char t[48]; snprintf(t, sizeof(t), "ERR prun %d\n", (int)e); reply(t); if (!b1) free(buf); return; }
    char h[200];
    snprintf(h, sizeof(h), "OK PRUN %u %u %u\n", 40000000u, ring, seconds);
    reply(h);
    uint32_t cons = 0, over = 0, x = 0; uint64_t tot = 0;
    int64_t tend = esp_timer_get_time() + (int64_t)seconds * 1000000, tnext = esp_timer_get_time() + 1000000;
    const char *why = "time";
    while (esp_timer_get_time() < tend) {
        uint32_t p = pc_rx.prod;
        if ((uint32_t)(p - cons) > ring) { over++; tot += p - cons; cons = p; }      /* producer lapped the consumer */
        while (cons != p) {                                                           /* consume in place */
            unsigned off = (unsigned)(tot % ring), k = p - cons;
            if (k > ring - off) k = ring - off;
            const uint32_t *w = (const uint32_t *)(buf + off);
            for (unsigned j = 0; j + 4 <= k; j += 4) x ^= w[j >> 2];
            cons += k; tot += k;
        }
        if (esp_timer_get_time() >= tnext) {
            tnext += 1000000;
            int64_t dt = pc_rx.t_last - pc_rx.t_first; uint32_t db = pc_rx.prod_last - pc_rx.prod_first;
            double ppm = dt > 0 ? ((double)db / ((double)dt * 40.0) - 1.0) * 1e6 : 0;
            snprintf(h, sizeof(h), "PST %" PRId64 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %.2f %08" PRIx32 "\n", esp_timer_get_time(), tot, pc_rx.nodes, pc_rx.eofs, over, ppm, x);
            reply(h);
        }
        char l[48]; if (burst_serial_poll_run_line(l, sizeof(l))) { why = "stop"; break; }
    }
    /* bytes between first and last node completion: 64-bit from the 32-bit counters (< 107 s per wrap of 2^32) */
    int64_t dt = pc_rx.t_last - pc_rx.t_first;
    double d32 = (double)(uint32_t)(pc_rx.prod_last - pc_rx.prod_first);
    double db = d32 + 4294967296.0 * floor(((double)dt * 40.0 - d32) / 4294967296.0 + 0.5);
    pc_close(); pc_unroute(); pc_modem_stop();
    unsigned bad = 0; for (unsigned j = 0; j < ng; j++) bad += guard[j] != (0x5a5aa5a5u ^ j);
    double exp_b = (double)dt * 40.0;
    snprintf(h, sizeof(h), "END PRUN %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRId64 " %.0f %.0f %.3f guard %u/%u %s\n", tot, pc_rx.nodes, pc_rx.eofs, over, dt, db, exp_b,
             exp_b > 0 ? (db / exp_b - 1.0) * 1e6 : 0.0, bad, ng, why);
    burst_serial_drain_stop();
    reply(h);
    if (!b1) free(buf);
}

/* ------------------------------------------------------------ commands */
static void pc_stat(void) {
    char h[320];
    if (!pe_have) { reply("PSTAT none\n"); return; }
    const pe_t *e = &pe_last; double sec = (e->t_end - e->t_start) * 1e-6, tot = (double)e->cons64;
    snprintf(h, sizeof(h), "PSTAT blocks %" PRIu32 " bytes %" PRIu64 " lost %" PRIu64 " overruns %" PRIu32 " late %" PRIu32 " duty %.4f maxlag %" PRIu32
             " lsb %u peak %u sec %.2f rate %.0f cpb %.2f\n", e->blocks, e->cons64, e->lost, e->overruns, e->late, tot > 0 ? 1.0 - (double)e->lost / tot : 0.0, e->maxlag,
             e->lsb, e->peak, sec, sec > 0 ? tot / sec : 0.0, e->blocks ? (double)e->work_cyc / ((double)e->blocks * PE_BLK) : 0.0);
    reply(h);
}
static bool pc_command(char *line) {
    unsigned a[8]; char x;
    if (sscanf(line, "PPINS %u %u %u %u %u %u %u %u %c", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &x) == 8) {
        for (unsigned k = 0; k < 8; k++) if (a[k] > 28 || (a[k] >= 11 && a[k] <= 22)) { reply("ERR ppins\n"); return true; }
        for (unsigned k = 0; k < 8; k++) pc_pins[k] = (int8_t)a[k];
        reply("OK\n"); return true;
    }
    if (sscanf(line, "PMAP %u %u %u %u %u %u %u %u %c", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &x) == 8) {
        for (unsigned k = 0; k < 8; k++) if (a[k] > 31) { reply("ERR pmap\n"); return true; }
        for (unsigned k = 0; k < 8; k++) pc_map[k] = (uint8_t)a[k];
        reply("OK\n"); return true;
    }
    if (!strcmp(line, "PLSB AUTO")) { pc_lsb = -1; reply("OK\n"); return true; }
    if (sscanf(line, "PLSB %u %c", &a[0], &x) == 1 && a[0] <= 6) { pc_lsb = (int)a[0]; pc_map_window(pc_map, a[0]); reply("OK\n"); return true; }
    if (!strcmp(line, "PLEVEL?")) {
        unsigned pk, l = pc_level(&pk); char h[48]; snprintf(h, sizeof(h), "PLEVEL %u %u\n", l, pk); reply(h); return true;
    }
    if (!strcmp(line, "PCFG?")) {
        char h[200];
        snprintf(h, sizeof(h), "PCFG pins %d %d %d %d %d %d %d %d map %u %u %u %u %u %u %u %u drive %u modem %u lsb %d edge %u\n", pc_pins[0], pc_pins[1], pc_pins[2], pc_pins[3],
                 pc_pins[4], pc_pins[5], pc_pins[6], pc_pins[7], pc_map[0], pc_map[1], pc_map[2], pc_map[3], pc_map[4], pc_map[5], pc_map[6], pc_map[7], pc_drive, pc_modem_flags,
                 pc_lsb, pc_edge);
        reply(h); return true;
    }
    if (sscanf(line, "PDRIVE %u %c", &a[0], &x) == 1 && a[0] <= 3) { pc_drive = a[0]; reply("OK\n"); return true; }
    if (sscanf(line, "PDC %u %c", &a[0], &x) == 1 && a[0] <= 1) { pc_dccal = a[0]; reply("OK\n"); return true; }
    if (sscanf(line, "PEDGE %u %c", &a[0], &x) == 1 && a[0] <= 1) { pc_edge = a[0]; reply("OK\n"); return true; }
    if (sscanf(line, "PMODEM %u %c", &a[0], &x) == 1 && a[0] <= 7) { pc_modem_flags = a[0]; reply("OK\n"); return true; }
    if (!strcmp(line, "PSTAT?")) { pc_stat(); return true; }
    if (!strncmp(line, "PDIAG", 5)) {
        unsigned n = 20000, div = 0; int m = sscanf(line, "PDIAG %u %u %c", &n, &div, &x);
        if (m > 2 || n < 100 || n > 1000000 || div > 5) { reply("ERR pdiag\n"); return true; }
        pc_diag_scan(n, div); return true;
    }
    if (!strncmp(line, "PCAP ", 5)) {
        unsigned n = 0, div = 0, where = 0;
        int m = sscanf(line, "PCAP %u %u %u %c", &n, &div, &where, &x);
        if (m < 1 || m > 3 || n < 256 || n > 65532 || (n & 3) || div > 5) { reply("ERR pcap_args\n"); return true; }
        pc_capture(n, div, where); return true;
    }
    if (!strncmp(line, "PRUN ", 5)) {
        unsigned sec = 0, ring = PE_RING;
        int m = sscanf(line, "PRUN %u %u %c", &sec, &ring, &x);
        if (m < 1 || m > 2 || !sec || sec > 3600) { reply("ERR prun_args\n"); return true; }
        pc_run(sec, ring); return true;
    }
    return false;
}
/* STREAM on the gap-free PARLIO ring (STRSRC 1).  Each 8192-byte block (40 MS/s,
 * 4+4 bit) is boxcar-summed by 16 (pc_box16) into 512 dump-format words at 2.5 MS/s, then CIC3 (R1) + DC blocker /
 * NCO + FIR (R2) exactly as stream_run; FIR outputs are collected across blocks into frames of >= 768 samples (or
 * 20 ms).  Same protocol as STREAM: "OK STREAM rate bits 2500000 R1 R2 taps 512", "BLK ...", gap_in in 2.5 MS/s
 * samples (0 unless the ring overran).  Level: 10-bit LSB units for the window lsb 6, 2^(6-lsb) higher below. */
static bool stream_run_p8(unsigned bits, unsigned R1, unsigned R2, int32_t offset_hz, uint64_t limit, bool dcblock) {
    dsp_init_tables();
    memset(st_cyc, 0, sizeof(st_cyc)); st_blocks = 0;
    static cic3_t cic; static firdec_t *fd; static pe_t pe;
    const unsigned words = PE_BLK / 16, fin = 2500000u, fm = fin / R1, out_rate = fm / R2, cap = 1024 + 160;
    uint32_t *cbuf = heap_caps_malloc(words * 4, MALLOC_CAP_8BIT);
    int32_t *mid = heap_caps_malloc((words + 8) * 2 * 4, MALLOC_CAP_8BIT), *outb = heap_caps_malloc(cap * 2 * 4, MALLOC_CAP_8BIT);
    uint8_t *pl = heap_caps_malloc(cap * 4, MALLOC_CAP_8BIT);
    fd = heap_caps_malloc(sizeof(firdec_t), MALLOC_CAP_8BIT);
    bool ring = cbuf && mid && outb && pl && fd && ring_open();
    if (!ring) {
        free(cbuf); free(mid); free(outb); free(pl); free(fd);
        char e[64]; snprintf(e, sizeof(e), "ERR stream_memory %u\n", (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); reply(e); return false;
    }
    unsigned gbits = 0; while ((1u << gbits) < R1 * R1 * R1) gbits++;
    unsigned taps = 16 * R2 + 1; if (taps > 129) taps = 129;
    fir_design(taps, 0.5f / R2, 7.0f);
    unsigned warm = taps / R2 + 2;
    uint32_t ph = 0, dph = (uint32_t)(int32_t)llroundf((float)offset_hz / fm * 4294967296.0f);
    int32_t dci = 0, dcq = 0;
    memset(&cic, 0, sizeof(cic)); cic.R = R1; cic.sh = gbits + 9 > 14 ? gbits + 9 - 14 : 0;
    memset(fd, 0, sizeof(*fd)); fd->R = R2;
    char h[160];
    if (pe_start(&pe)) { ring_close(); free(cbuf); free(mid); free(outb); free(pl); free(fd); reply("ERR stream_parlio\n"); return false; }
    snprintf(h, sizeof(h), "OK STREAM %u %u %u %u %u %u %u\n", out_rate, bits, fin, R1, R2, taps, words);
    reply(h);
    uint64_t out_idx = 0, produced = 0; uint32_t seq = 0, overruns = 0, pending_gap = 0; unsigned nacc = 0;
    int64_t frame_t = 0, frame_wall = 0;
    const char *why = "stop";
    int64_t last_progress = esp_timer_get_time(); unsigned last_tail = 0;
    for (;;) {
        uint32_t c0 = esp_cpu_get_cycle_count(), gb; int64_t t16;
        const uint8_t *x = pe_next(&pe, &gb, &t16);
        if (!x) { why = "capture_error"; break; }
        pc_box16(x, words, cbuf);
        pe_done(&pe);
        uint32_t c1 = esp_cpu_get_cycle_count(); st_cyc[0] += c1 - c0;
        uint32_t gap_in = gb / 16;
        if (gap_in) ph += (uint32_t)(((int64_t)(int32_t)dph * gap_in) / R1);   /* NCO phase-coherent across a gap */
        pending_gap += gap_in;
        int64_t blk_t0 = t16 / 16;
        uint32_t i0 = (R1 - cic.k - 1) + R1 * (R2 - fd->k - 1);               /* input index of this block's first output */
        unsigned k = cic3_run(&cic, cbuf, words, mid);
        for (unsigned j = 0; j < k; j++) {
            int32_t I = mid[2 * j], Q = mid[2 * j + 1];
            if (dcblock) { dci += ((I << 8) - dci) >> 10; dcq += ((Q << 8) - dcq) >> 10; I -= dci >> 8; Q -= dcq >> 8; }
            if (dph) {
                int32_t cs = sin_tab[((ph >> 22) + 256) & 1023], sn = sin_tab[ph >> 22];
                int32_t I2 = (I * cs - Q * sn) >> 14, Q2 = (I * sn + Q * cs) >> 14; I = I2; Q = Q2; ph += dph;
            }
            mid[2 * j] = I; mid[2 * j + 1] = Q;
        }
        uint32_t c2 = esp_cpu_get_cycle_count(); st_cyc[1] += c2 - c1;
        unsigned n = firdec_run(fd, mid, k, outb + 2 * nacc);
        if (!nacc && n) { frame_t = blk_t0 + (int64_t)i0 * 1000000 / fin; frame_wall = esp_timer_get_time(); }
        nacc += n;
        uint32_t c3 = esp_cpu_get_cycle_count(); st_cyc[2] += c3 - c2;
        if (nacc >= 768 || (nacc && esp_timer_get_time() - frame_wall > 20000)) {
            unsigned skip = produced < warm ? (unsigned)(warm - produced < nacc ? warm - produced : nacc) : 0;
            produced += nacc;
            pending_gap += skip * R1 * R2;
            unsigned m = nacc - skip;
            if (limit && out_idx + m > limit) m = (unsigned)(limit - out_idx);
            unsigned bytes = m * (bits / 4);
            for (unsigned j = 0; j < m; j++) {
                int32_t I = outb[2 * (j + skip)], Q = outb[2 * (j + skip) + 1];
                if (bits == 16) {
                    I = I > 32767 ? 32767 : I < -32768 ? -32768 : I; Q = Q > 32767 ? 32767 : Q < -32768 ? -32768 : Q;
                    ((int16_t *)pl)[2 * j] = I; ((int16_t *)pl)[2 * j + 1] = Q;
                } else {
                    I >>= 7; Q >>= 7;
                    ((int8_t *)pl)[2 * j] = I > 127 ? 127 : I < -128 ? -128 : I; ((int8_t *)pl)[2 * j + 1] = Q > 127 ? 127 : Q < -128 ? -128 : Q;
                }
            }
            int64_t t_first = frame_t + (int64_t)skip * R1 * R2 * 1000000 / fin;
            if (m) {
                uint32_t crc = esp_rom_crc32_le(0, pl, bytes);
                int hl = snprintf(h, sizeof(h), "BLK %" PRIu32 " %" PRIu64 " %u %08" PRIx32 " %" PRId64 " %" PRIu32 "\n", seq, out_idx, m, crc, t_first, pending_gap);
                if (!ring_frame(h, hl, pl, bytes)) overruns++;   /* transport too slow: frame dropped, seq/idx gap */
                seq++; out_idx += m; pending_gap = 0;
            }
            nacc = 0;
            st_cyc[3] += esp_cpu_get_cycle_count() - c3;
            if (limit && out_idx >= limit) { why = "limit"; break; }
        }
        ring_pump();
        uint32_t c4 = esp_cpu_get_cycle_count();
        st_cyc[4] += c4 - c0; pe.work_cyc += c4 - c0; st_blocks++;
        if (pc_poll_stop()) break;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    pe_stop(&pe);
    ring_close();
    snprintf(h, sizeof(h), "END STREAM %" PRIu32 " %" PRIu64 " %" PRIu64 " %" PRIu32 " %s\n", seq, out_idx, pe.lost / 16, overruns, why);
    burst_serial_drain_stop();
    reply(h);
    free(cbuf); free(mid); free(outb); free(pl); free(fd);
    return true;
}

/* VFM on the gap-free PARLIO path (VFMSRC 1, default).  Same commands and
 * frames as vfm_run, so the app needs no change.  Differences: "OK VFM" reports fin 20000000 (the bins have the
 * 20 MS/s scale) and words = PE_BLK (bytes per block at 40 MS/s); gap_in / lost_in are bytes (= 40 MS/s samples,
 * 0 unless the ring overran); one VFL frame per PFR blocks (0.82 ms) to keep the header overhead low; a full USB
 * ring drops the frame (counted as overrun) instead of pausing the receiver. */
#ifndef ESPSOUP_VFM_PARLIO_DEFAULT
#define ESPSOUP_VFM_PARLIO_DEFAULT 1
#endif
static unsigned vfm_src = ESPSOUP_VFM_PARLIO_DEFAULT;   /* VFMSRC: 1 = PARLIO, 0 = dump engine (c5_demod.h) */
#define PFR 4u
static bool vfm_run_p8(vf_t *s) {
    s->fin = 20000000; s->src = 1;
    if (!vf_init(s)) { reply("ERR vfm_args\n"); return false; }
    dm_release();
    const unsigned rec = VF_REC + s->px, maxl = PFR * 6;
    uint8_t *tab = dm_fast_alloc(256), *out = dm_fast_alloc(maxl * rec);
    int16_t *bins = dm_fast_alloc((VFP_KEEP + PE_BLK / 8 + 4) * sizeof(int16_t));
    vf_run_t *runs = heap_caps_malloc(384 * sizeof(vf_run_t), MALLOC_CAP_8BIT);
    bool ring = tab && out && bins && runs && ring_open_sz(16384);
    if (!ring) {
        free(tab); free(out); free(bins); free(runs);
        char e[64]; snprintf(e, sizeof(e), "ERR vfm_memory %u\n", (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); reply(e); return false;
    }
    memcpy(tab, vf_ptab, 256);
    s->tab = tab; s->bins = bins; s->runs = runs; s->maxruns = 384;
    static pe_t e;
    char h[160];
    pc_level_robust = true;
    esp_err_t pe_err = pe_start(&e);
    pc_level_robust = false;
    if (pe_err) { ring_close(); free(tab); free(out); free(bins); free(runs); reply("ERR vfm_parlio\n"); return false; }
    snprintf(h, sizeof(h), "OK VFM %u %u %u %u %u %u %u %u\n", s->fin, s->D, s->px, s->std_force, s->pol_force, s->sel, s->rate_Bps, PE_BLK);
    reply(h);
    uint32_t seq = 0, overruns = 0, pend_gap = 0, idle = 0; unsigned fnl = 0, fb = 0; int64_t ft_us = 0;
    const char *why = "stop";
    int64_t last_progress = esp_timer_get_time(); unsigned last_tail = 0;
    for (;;) {
        uint32_t gap; int64_t t16;
        const uint8_t *x = pe_next(&e, &gap, &t16);
        if (!x) { why = "capture_error"; break; }
        uint32_t c1 = esp_cpu_get_cycle_count();
        if (!fb) ft_us = t16 / 16;
        pend_gap += gap;
        fnl += vf_block_p8(s, x, PE_BLK, gap, t16, out + fnl * rec, maxl - fnl);
        pe_done(&e);
        if (++fb >= PFR || fnl + 6 > maxl) {
            if (fnl || ++idle >= 12) {        /* idle frames (nlines 0) keep the host informed while unlocked */
                unsigned bytes = fnl * rec;
                uint32_t crc = esp_rom_crc32_le(0, out, bytes);
                int hl = snprintf(h, sizeof(h), "VFL %" PRIu32 " %" PRId64 " %u %u %" PRIu32 " %08" PRIx32 " %" PRId32 " %" PRId32 " %u\n", seq, ft_us, fnl, s->px,
                                  pend_gap, crc, vf_khz(s, s->tip16), vf_khz(s, s->blank16), vf_state(s));
                if (!ring_frame(h, hl, out, bytes)) overruns++;
                seq++; pend_gap = 0; idle = 0;
            }
            fnl = 0; fb = 0;
        }
        e.work_cyc += esp_cpu_get_cycle_count() - c1;
        char l[48]; if (burst_serial_poll_run_line(l, sizeof(l))) break;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    pe_stop(&e);
    ring_close();
    vf_last = *s; vf_have = true;
    snprintf(h, sizeof(h), "END VFM %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %s\n", seq, e.blocks, e.lost, overruns,
             s->lines, s->l_out, s->l_drop_rate, why);
    burst_serial_drain_stop();
    reply(h);
    free(tab); free(bins); free(runs); free(out);
    return true;
}

/* STRSRC 1: R1 * R2 = 2.5e6 / out (R2 2, 4, 8 or 5; R1 2..128: the CIC sees 2.5 MS/s) -> e.g. 250k, 125k, 100k, 50k,
 * 25k, 20k, 10k; not 200k / 500k (no integer split). */
static unsigned st_src;
static bool stream_plan_p8(unsigned out, unsigned *R1, unsigned *R2) {
    static const unsigned r2s[4] = {2, 4, 8, 5};
    for (unsigned j = 0; j < 4; j++) {
        unsigned r2 = r2s[j];
        if (2500000u % (out * r2) == 0) { unsigned r1 = 2500000u / (out * r2); if (r1 >= 2 && r1 <= 128) { *R1 = r1; *R2 = r2; return true; } }
    }
    return false;
}

/* ------------------------------------------------------------ commands */
/* Runs before demod_command / cont_command: with VFMSRC 1 (default) VFM runs on the PARLIO ring, with STRSRC 1
 * (and no explicit hardware rate) STREAM8/16 does; everything else falls through to the dump-engine handlers. */
static bool parlio_command(char *line) {
    unsigned n, rate, bits = 0; char extra; int32_t off = 0; int m;
    if (line[0] == 'P' && pc_command(line)) return true;
    if (sscanf(line, "VFMSRC %u %c", &n, &extra) == 1 && n <= 1) { vfm_src = n; reply("OK\n"); return true; }
    if (sscanf(line, "STRSRC %u %c", &n, &extra) == 1 && n <= 1) { st_src = n; reply("OK\n"); return true; }
    if (!strcmp(line, "STRSRC?")) { reply(st_src ? "STRSRC 1 PARLIO\n" : "STRSRC 0 DUMP\n"); return true; }
    if (!strcmp(line, "VFMSRC?")) { reply(vfm_src ? "VFMSRC 1 PARLIO\n" : "VFMSRC 0 DUMP\n"); return true; }
    if (vfm_src && !strncmp(line, "VFM ", 4)) {
        unsigned div; int nn = 0; static vf_t vs;
        if (sscanf(line + 4, "%u%n", &div, &nn) != 1 || (div != 2 && div != 3) || !vf_parse(line + 4 + nn, &vs)) { reply("ERR vfm_args\n"); return true; }
        vfm_run_p8(&vs);
        return true;
    }
    uint32_t lim32 = 0; unsigned dcb = 1, hwi = 99;
    if (st_src && (((m = sscanf(line, "STREAM8 %u %" SCNd32 " %" SCNu32 " %u %u %c", &rate, &off, &lim32, &dcb, &hwi, &extra)) >= 1 && m <= 5 && (bits = 8)) ||
                   ((m = sscanf(line, "STREAM16 %u %" SCNd32 " %" SCNu32 " %u %u %c", &rate, &off, &lim32, &dcb, &hwi, &extra)) >= 1 && m <= 5 && (bits = 16)))) {
        if (hwi != 99) return false;             /* explicit hardware rate: dump engine (cont_command) */
        if (m < 2) off = 0;
        if (m < 3) lim32 = 0;
        if (m < 4) dcb = 1;
        unsigned R1, R2;   /* gap-free PARLIO source, 2.5 MS/s after the boxcar-16 */
        bool ok = rate >= 1000 && rate <= 500000 && stream_plan_p8(rate, &R1, &R2) && off <= (int32_t)(2500000 / R1 / 4) && off >= -(int32_t)(2500000 / R1 / 4);
        if (!ok) { reply("ERR stream_args\n"); return true; }
        stream_run_p8(bits, R1, R2, off, lim32, dcb != 0);
        return true;
    }
    if (!strcmp(line, "PBENCH")) {   /* diagnostic: vf_disc_p8 cycles per byte, data in bank 1 (live GDMA ring or idle) vs heap */
        uint8_t *hb = heap_caps_malloc(PE_BLK, MALLOC_CAP_DMA | MALLOC_CAP_8BIT); int16_t *vb = heap_caps_malloc((PE_BLK / 8 + 4) * 2, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        uint8_t *tb = heap_caps_malloc(256, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!hb || !vb || !tb) { free(hb); free(vb); free(tb); reply("ERR pbench_memory\n"); return true; }
        memcpy(tb, vf_ptab, 256); memcpy(hb, PC_BANK1, PE_BLK);
        unsigned pp = 0; uint32_t c0, cyc[3] = {0, 0, 0};
        c0 = esp_cpu_get_cycle_count(); vf_disc_p8(PC_BANK1, PE_BLK / 8, tb, &pp, vb); cyc[0] = esp_cpu_get_cycle_count() - c0;
        c0 = esp_cpu_get_cycle_count(); vf_disc_p8(hb, PE_BLK / 8, tb, &pp, vb); cyc[1] = esp_cpu_get_cycle_count() - c0;
        static pe_t be; uint32_t live = 0;
        if (!pe_start(&be)) {   /* while the GDMA writes the ring */
            uint32_t g; int64_t t; const uint8_t *x = pe_next(&be, &g, &t);
            if (x) { c0 = esp_cpu_get_cycle_count(); vf_disc_p8(x, PE_BLK / 8, tb, &pp, vb); live = esp_cpu_get_cycle_count() - c0;
                     c0 = esp_cpu_get_cycle_count(); vf_disc_p8(hb, PE_BLK / 8, tb, &pp, vb); cyc[2] = esp_cpu_get_cycle_count() - c0; }
            pe_stop(&be);
        }
        char h[160]; snprintf(h, sizeof(h), "PBENCH bank1_idle %.2f heap %.2f bank1_live %.2f heap_live %.2f cyc/byte\n", cyc[0] / (double)PE_BLK, cyc[1] / (double)PE_BLK,
                              live / (double)PE_BLK, cyc[2] / (double)PE_BLK);
        reply(h); free(hb); free(vb); free(tb); return true;
    }
    return false;
}
