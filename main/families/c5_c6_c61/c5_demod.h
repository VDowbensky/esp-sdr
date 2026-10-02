/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * ESP32-C5 on-device demodulators (ESPsoup extension, receive only), included
 * by c5_opt.h: VFM (FM video -> line stream, CAPS DEMODFM) and GFSK (packet
 * catcher with BLE de-whitening/CRC, CAPS GFSKPKT) on the continuous engine.
 * The DSP lives in demod_dsp.h (integer only, compiled unchanged on the host
 * by test/demod for bit-exact test vectors); it runs from flash like CSPEC.
 * Protocol and measurements: docs/espsoup-c5.md. */
#define DM_ATTR
#define DM_CYC() esp_cpu_get_cycle_count()
#define VFP_ATTR IRAM_ATTR
#include "demod_dsp.h"
static vf_t vf_last; static gf_t gf_last;            /* DSTAT? */
/* VFM colour (VFMCOLOR): the request of the last VFM line (nseg 0 = off) and the statistics of the last run */
static vc_t vc_cfg; static vc_t vc_last; static bool vc_have;
/* test-vector buffers (DMLOAD/DMVFM/DMGF); released before any engine run needs the heap */
static uint32_t *dm_up; static vf_t dm_vs; static gf_t dm_gs; static int16_t *dm_bins; static vf_run_t *dm_runs;
/* hot DSP buffers: main SRAM first (MALLOC_CAP_DMA excludes the slow LP SRAM heap), else anywhere */
static bool vf_bins_lp;
static void *dm_fast_alloc(size_t n) {
    void *p = heap_caps_malloc(n, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    return p ? p : heap_caps_malloc(n, MALLOC_CAP_8BIT);
}
static uint8_t *dm_tab; static unsigned dm_pol = 1;  /* DMPOL: bit0 table copy in SRAM (= vfm_run layout), bit1 bins in LP SRAM (layout experiments) */
static void dm_release(void) { free(dm_up); free(dm_bins); free(dm_runs); free(dm_tab); dm_up = NULL; dm_bins = NULL; dm_runs = NULL; dm_tab = NULL; dm_vs.fin = 0; dm_gs.fin = 0; }
static bool vf_have, gf_have;
/* The 32 KB test-vector buffer only lives across DMLOAD / DMVFM / DMGF sequences:
 * called for every command before any handler can need the heap. */
static void dm_release_unless_vector(const char *line) {
    if (dm_up && strncmp(line, "DMLOAD ", 7) && strncmp(line, "DMVFM ", 6) && strncmp(line, "DMGF ", 5)) dm_release();
}

static int32_t vf_khz(const vf_t *s, int32_t z16) {   /* z level (Q4) -> host-orientation kHz */
    return (int32_t)(-(int64_t)s->pol * z16 * (int64_t)s->fin / (16 * 1024 * 1000LL));
}
static unsigned vf_state(const vf_t *s) { return (s->locked ? 1u : 0) | (s->vlocked ? 2u : 0) | (s->std == 2 ? 4u : 0) | (s->pol > 0 ? 8u : 0); }
/* VFM: continuous FM-video line stream.  Frames "VFL seq t_us nlines px gap crc tip_khz blank_khz st"
 * + nlines x (8-byte record header + px luminance bytes), see docs/espsoup-c5.md. */
static bool vfm_run(unsigned div, vf_t *s) {
    s->fin = hw_rates[div];
    if (!vf_init(s)) { reply("ERR vfm_args\n"); return false; }
    if (vc_cfg.nseg) { reply("ERR vfm_color_needs_parlio\n"); return false; }   /* colour only on the PARLIO source (VFMSRC 1) */
    unsigned words = CONT_WORDS; bool ring = false; uint32_t *cbuf = NULL; uint8_t *tab = NULL, *out = NULL; int16_t *bins = NULL; vf_run_t *runs = NULL;
    unsigned maxl = 0;
    dm_release();
    for (; words >= CONT_WORDS / 2; words /= 2) {
        maxl = words / (s->B * 300) + 2;
        /* main SRAM layout (measured on the C5): the arctan table and the bins must not live in flash/LP SRAM
         * (+1.5 / +2..3 cycles per sample); an 8 KB USB ring is plenty at <= 850 kB/s and leaves room for them */
        cbuf = heap_caps_malloc(words * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ring = cbuf && ring_open_sz(8192);
        tab = dm_fast_alloc(4096);
        bins = dm_fast_alloc((words / s->B + 4) * 2);
        out = dm_fast_alloc(maxl * (VF_REC + s->px));
        runs = heap_caps_malloc(384 * sizeof(vf_run_t), MALLOC_CAP_8BIT);       /* acquisition only: LP SRAM is fine */
        if (cbuf && ring && tab && bins && runs && out) break;
        if (ring) ring_close();
        free(cbuf); free(tab); free(bins); free(runs); free(out); cbuf = NULL; ring = false;
    }
    if (!cbuf) { char e[64]; snprintf(e, sizeof(e), "ERR vfm_memory %u\n", (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); reply(e); return false; }
    memcpy(tab, vf_atan_tab, 4096);
    vf_bins_lp = (uintptr_t)bins >= 0x50000000u || (uintptr_t)tab >= 0x50000000u;   /* LP SRAM (slow) */
    s->tab = tab; s->bins = bins; s->runs = runs; s->maxruns = 384;
    char h[160];
    snprintf(h, sizeof(h), "OK VFM %u %u %u %u %u %u %u %u\n", s->fin, s->D, s->px, s->std_force, s->pol_force, s->sel, s->rate_Bps, words);
    reply(h);
    static cont_t c;
    cont_start(&c, div, cbuf, words);
    uint32_t seq = 0, overruns = 0, pend_gap = 0, idle = 0; bool stop = false; const char *why = "stop";
    int64_t last_progress = esp_timer_get_time(); unsigned last_tail = 0;
    while (!stop) {
        while (!stop && ring_used() > st_ring_size / 2) {
            ring_pump();
            if (cont_poll(&c)) stop = true;
            if (st_tail != last_tail) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
            if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; stop = true; }
        }
        if (stop) break;
        if (!cont_next(&c)) { why = "capture_error"; break; }
        uint32_t c1 = esp_cpu_get_cycle_count();
        unsigned nl = vf_block(s, cbuf, words, c.blk_gap, c.blk_t0, out, maxl);
        pend_gap += c.blk_gap;
        if (nl || ++idle >= 50) {           /* idle frames (nlines 0) keep the host informed while unlocked */
            unsigned bytes = nl * (VF_REC + s->px);
            uint32_t crc = esp_rom_crc32_le(0, out, bytes);
            int hl = snprintf(h, sizeof(h), "VFL %" PRIu32 " %" PRId64 " %u %u %" PRIu32 " %08" PRIx32 " %" PRId32 " %" PRId32 " %u\n", seq, c.blk_t0, nl, s->px,
                              pend_gap, crc, vf_khz(s, s->tip16), vf_khz(s, s->blank16), vf_state(s));
            if (!ring_frame(h, hl, out, bytes)) overruns++;
            seq++; pend_gap = 0; idle = 0;
        }
        c.work_cyc += esp_cpu_get_cycle_count() - c1;
        if (cont_poll(&c)) stop = true;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    cont_stop(&c);
    ring_close();
    vf_last = *s; vf_have = true;
    snprintf(h, sizeof(h), "END VFM %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %s\n", seq, c.blocks, c.lost, overruns,
             s->lines, s->l_out, s->l_drop_rate, why);
    burst_serial_drain_stop();
    reply(h);
    free(cbuf); free(tab); free(bins); free(runs); free(out);
    return true;
}

/* GFSK: continuous packet catcher.  "GPK seq t_us n flags rssi10 cfo_hz dev_hz errs crc" + n bytes. */
static bool gfsk_run(unsigned div, gf_t *g) {
    g->fin = hw_rates[div];
    if (!gf_init(g)) { reply("ERR gfsk_args\n"); return false; }
    unsigned words = CONT_WORDS; bool ring = false; uint32_t *cbuf = NULL, *hits = NULL; gf_pkt_t *pk = NULL; uint8_t *pool = NULL, *bits = NULL; int32_t *sb = NULL;
    dm_release();
    for (; words >= CONT_WORDS / 2; words /= 2) {
        cbuf = heap_caps_malloc(words * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ring = cbuf && ring_open();
        sb = dm_fast_alloc(GF_SBUF * 4);
        bits = dm_fast_alloc(GF_MAXBITS);
        hits = dm_fast_alloc(256 * 4);
        pk = heap_caps_malloc(48 * sizeof(gf_pkt_t), MALLOC_CAP_8BIT);
        pool = heap_caps_malloc(4096, MALLOC_CAP_8BIT);
        if (cbuf && ring && hits && pk && pool && bits && sb) break;
        if (ring) ring_close();
        free(cbuf); free(hits); free(pk); free(pool); free(bits); free(sb); cbuf = NULL; ring = false;
    }
    if (!cbuf) { char e[64]; snprintf(e, sizeof(e), "ERR gfsk_memory %u\n", (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); reply(e); return false; }
    g->bits = bits; g->sbuf = sb;
    char h[160];
    snprintf(h, sizeof(h), "OK GFSK %u %u %u %u %u %08" PRIx32 " %u %u %u %d %u\n", g->fin, g->R, g->fin / g->R, g->sps, g->mode, g->sync, g->nbits, g->maxerr,
             g->len, g->chan, words);
    reply(h);
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
        uint32_t c1 = esp_cpu_get_cycle_count();
        unsigned np = gf_block(g, cbuf, words, hits, 256, pk, 48, pool, 4096);
        for (unsigned j = 0; j < np; j++) {
            const gf_pkt_t *p = &pk[j];
            int64_t t = c.blk_t0 + (int64_t)((uint64_t)p->pos * 1000000u / g->fin);
            uint32_t crc = esp_rom_crc32_le(0, pool + p->off, p->n);
            int hl = snprintf(h, sizeof(h), "GPK %" PRIu32 " %" PRId64 " %u %u %d %" PRId32 " %" PRId32 " %u %08" PRIx32 "\n", seq, t, p->n, p->flags, p->rssi10,
                              p->cfo_hz, p->dev_hz, p->errs, crc);
            if (!ring_frame(h, hl, pool + p->off, p->n)) overruns++;
            seq++;
        }
        c.work_cyc += esp_cpu_get_cycle_count() - c1;
        if (cont_poll(&c)) stop = true;
        if (st_tail != last_tail || !ring_used()) { last_tail = st_tail; last_progress = esp_timer_get_time(); }
        if (esp_timer_get_time() - last_progress > 2000000) { why = "stalled"; break; }
    }
    cont_stop(&c);
    ring_close();
    g->bits = NULL; g->sbuf = NULL; gf_last = *g; gf_have = true;
    snprintf(h, sizeof(h), "END GFSK %" PRIu32 " %" PRIu32 " %" PRIu64 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %s\n", seq, c.blocks, c.lost, overruns,
             g->hits, g->pkts, g->crc_ok, why);
    burst_serial_drain_stop();
    reply(h);
    free(cbuf); free(hits); free(pk); free(pool); free(bits); free(sb);
    return true;
}
/* BLE whitening channel index of the tuned frequency (advertising 37/38/39, data 0..36) */
static int ble_chan_of(unsigned mhz) {
    if (mhz == 2402) return 37;
    if (mhz == 2426) return 38;
    if (mhz == 2480) return 39;
    if (mhz >= 2404 && mhz <= 2424 && !(mhz & 1)) return (int)(mhz - 2404) / 2;
    if (mhz >= 2428 && mhz <= 2478 && !(mhz & 1)) return (int)(mhz - 2428) / 2 + 11;
    return 37;
}
/* GFSK argument parser shared by GFSK and GFDATA (after the leading fields) */
static bool gf_parse(const char *args, unsigned div, gf_t *g) {
    unsigned R = 0, kbps = 0, mode = 0, len = 0, nb = 0, me = 1, ok = 0; uint32_t sync = 0; int ch = -1; char x;
    int m = sscanf(args, "%u %u %u %u %" SCNx32 " %u %u %d %u %c", &R, &kbps, &mode, &len, &sync, &nb, &me, &ch, &ok, &x);
    if (m < 3 || m > 9) return false;
    memset(g, 0, sizeof(*g));
    if (!R || !kbps) return false;
    unsigned fin = hw_rates[div];
    if (fin % R || (fin / R) % (kbps * 1000)) return false;
    g->R = R; g->sps = fin / R / (kbps * 1000); g->mode = mode;
    g->len = m >= 4 ? len : (mode == 0 ? 37 : 32);
    g->sync = m >= 5 ? sync : (mode == 0 ? 0x8e89bed6u : 0);
    g->nbits = m >= 6 ? nb : (mode == 2 ? 16 : 32);
    g->maxerr = m >= 7 ? me : 1;
    g->chan = (m >= 8 && ch >= 0) ? ch : ble_chan_of(frequency_mhz);
    g->okonly = m >= 9 ? ok : 0;
    if (mode == 1 && m < 5) return false;          /* sync word required */
    if (div == 2) return R >= 4;                   /* 20 MS/s: CPU only with R >= 4 */
    return div >= 3 && div <= 5;
}
/* VFM argument parser: <D> <px> <std> <pol(host: 1 tip lowest, 2 tip highest)> <sel> <kBps> [<cseg> [<every> [<step>]]]
 * cseg > 0 requests colour (VFMCOLOR, vc_cfg): cseg U/V pairs per line, every n-th line (0 = automatic), phase of
 * every step-th byte (1, 2, 4; default 2).  Without cseg the run is exactly the old one. */
static bool vf_parse(const char *args, vf_t *s) {
    unsigned D = 0, px = 128, st = 0, pol = 0, sel = 1, kb = 850, cs = 0, ce = 0, cst = 2; char x;
    int m = sscanf(args, "%u %u %u %u %u %u %u %u %u %c", &D, &px, &st, &pol, &sel, &kb, &cs, &ce, &cst, &x);
    if (m > 9 || st > 2 || pol > 2 || sel > 3) return false;
    if (cs && (cs < 4 || cs > 64 || ce > 625 || (cst != 1 && cst != 2 && cst != 4))) return false;
    memset(s, 0, sizeof(*s));
    s->D = D; s->px = px; s->std_force = st; s->pol_force = pol == 1 ? 2 : pol == 2 ? 1 : 0; s->sel = sel; s->rate_Bps = kb * 1000;
    memset(&vc_cfg, 0, sizeof(vc_cfg)); vc_cfg.nseg = cs; vc_cfg.every = ce; vc_cfg.step = cst;
    return true;
}
static void dm_stat(void) {
    char h[600]; int o = snprintf(h, sizeof(h), "DSTAT");
    if (vf_have) {
        const vf_t *s = &vf_last; double ns = s->samples ? (double)s->samples : 1;
        o += snprintf(h + o, sizeof(h) - o, " vfm blocks %" PRIu32 " lines %" PRIu32 " sync %" PRIu32 " fly %" PRIu32 " out %" PRIu32 " drop_rate %" PRIu32 " drop_cut %" PRIu32
                      " vbi %" PRIu32 " vsyncs %" PRIu32 " relocks %" PRIu32 " acq %" PRIu32 " pol %d std %d qs %u tip_khz %" PRId32 " blank_khz %" PRId32 " T_ns %" PRId32 " bins_lp %d"
                      " cps_disc %.2f cps_sync %.2f cps_slice %.2f",
                      s->blocks, s->lines, s->l_sync, s->l_fly, s->l_out, s->l_drop_rate, s->l_drop_cut, s->l_vbi, s->vsyncs, s->relocks, s->acq, -s->pol, s->std, s->qs,
                      vf_khz(s, s->tip16), vf_khz(s, s->blank16), (int32_t)((int64_t)s->T16 * 200 / 65536), (int)vf_bins_lp, s->cyc_disc / ns, s->cyc_sync / ns, s->cyc_slice / ns);
        if (vc_have && s->vc) {
            const vc_t *c = &vc_last;
            o += snprintf(h + o, sizeof(h) - o, " color lines %" PRIu32 " ref2 %" PRIu32 " weak %" PRIu32 " drop_rate %" PRIu32 " drop_full %" PRIu32 " drop_cut %" PRIu32
                          " pre %" PRIu32 " nseg %u every %u step %u cps_color %.2f cyc_line %.0f",
                          c->lines, c->ref2, c->weak, c->drop_rate, c->drop_full, c->drop_cut, c->pre, c->nseg, c->every, c->step, s->cyc_color / ns,
                          (c->lines + c->pre) ? (double)s->cyc_color / (c->lines + c->pre) : 0.0);
        }
    }
    if (gf_have) {
        const gf_t *g = &gf_last; double ns = g->samples ? (double)g->samples : 1;
        o += snprintf(h + o, sizeof(h) - o, " gfsk blocks %" PRIu32 " hits %" PRIu32 " hit_ovf %" PRIu32 " pkts %" PRIu32 " crc_ok %" PRIu32 " trunc %" PRIu32
                      " rejected %" PRIu32 " pool_ovf %" PRIu32 " cps_scan %.2f cps_dec %.2f",
                      g->blocks, g->hits, g->hit_ovf, g->pkts, g->crc_ok, g->trunc, g->rejected, g->pool_ovf, g->cyc_scan / ns, g->cyc_dec / ns);
    }
    snprintf(h + o, sizeof(h) - o, "\n"); reply(h);
}

/* ------------------------------------------------------------ commands */
static bool demod_command(const char *line) {
    unsigned n; char extra;
    if (sscanf(line, "DMPOL %u %c", &n, &extra) == 1) { dm_pol = n; reply("OK\n"); return true; }
    /* VFM <rate_idx 2|3> [<D> [<px> [<std> [<pol> [<sel> [<kBps>]]]]]]: FM video line stream (DEMODFM) */
    if (!strncmp(line, "VFM ", 4)) {
        unsigned div; int nn = 0; static vf_t vs;
        if (sscanf(line + 4, "%u%n", &div, &nn) != 1 || (div != 2 && div != 3) || !vf_parse(line + 4 + nn, &vs)) { reply("ERR vfm_args\n"); return true; }
        vfm_run(div, &vs);
        return true;
    }
    /* GFSK <rate_idx 2..5> <R> <kbps> <mode> [<len> [<sync_hex> [<nbits> [<maxerr> [<chan> [<okonly>]]]]]] (GFSKPKT) */
    if (!strncmp(line, "GFSK ", 5)) {
        unsigned div; int nn = 0; static gf_t gs;
        if (sscanf(line + 5, "%u%n", &div, &nn) != 1 || div > 5 || !gf_parse(line + 5 + nn, div, &gs)) { reply("ERR gfsk_args\n"); return true; }
        gfsk_run(div, &gs); return true;
    }
    if (!strcmp(line, "DSTAT?")) { dm_stat(); return true; }
    /* VFMDATA <rate_idx> <n> [VFM args]: snapshot (DATAT, CAPT20 payload) + "VFX nlines px crc" + the line
     * records vf_block computes from exactly these samples (one block, no rate limit) - bit-exact check */
    if (!strncmp(line, "VFMDATA ", 8)) {
        unsigned div; int nn = 0; static vf_t vs;
        if (sscanf(line + 8, "%u %u%n", &div, &n, &nn) != 2 || (div != 2 && div != 3) || n < 2048 || n > 16380 || !vf_parse(line + 8 + nn, &vs)) { reply("ERR vfmdata_args\n"); return true; }
        vs.rate_Bps = 0; vs.fin = hw_rates[div];
        if (!vf_init(&vs)) { reply("ERR vfmdata_args\n"); return true; }
        n -= n % vs.B;
        unsigned maxl = n / (vs.B * 300) + 2;
        int16_t *bins = heap_caps_malloc((n / vs.B + 4) * 2, MALLOC_CAP_8BIT); vf_run_t *runs = heap_caps_malloc(384 * sizeof(vf_run_t), MALLOC_CAP_8BIT);
        uint8_t *out = heap_caps_malloc(maxl * (VF_REC + vs.px), MALLOC_CAP_8BIT);
        if (!bins || !runs || !out) { free(bins); free(runs); free(out); reply("ERR mem\n"); return true; }
        vs.tab = vf_atan_tab; vs.bins = bins; vs.runs = runs; vs.maxruns = 384;
        prepare_rx(); uint32_t el;
        const char *err = fast_capture(n, div, &el);
        if (!err) {
            unsigned nl = vf_block(&vs, DUMP_BUF, n, 0, opt_last_t0, out, maxl), bytes = nl * (VF_REC + vs.px);
            opt_pack(DUMP_BUF, n, 20);
            if (opt_send_frame(DUMP_BUF, opt_wire(n, 20), n, el, true)) {
                char h[160]; snprintf(h, sizeof(h), "VFX %u %u %08" PRIx32 " %d %d %u %" PRId32 " %" PRId32 "\n", nl, vs.px, esp_rom_crc32_le(0, out, bytes),
                                      vs.pol, vs.std, vs.qs, vs.tip16, vs.blank16);
                (void)(send_bytes(h, strlen(h)) && send_bytes(out, bytes));
            }
        } else reply(err);
        free(bins); free(runs); free(out); return true;
    }
    /* GFDATA <rate_idx> <n> <GFSK args>: snapshot + "GFX npk bytes crc" + records
     * [u32 pos][u16 n][u8 flags][u8 errs][i16 rssi10][i32 cfo][i32 dev] + n bytes */
    if (!strncmp(line, "GFDATA ", 7)) {
        unsigned div; int nn = 0; static gf_t gs;
        if (sscanf(line + 7, "%u %u%n", &div, &n, &nn) != 2 || div > 5 || n < 1024 || n > 16380 || !gf_parse(line + 7 + nn, div, &gs)) { reply("ERR gfdata_args\n"); return true; }
        gs.fin = hw_rates[div];
        if (!gf_init(&gs)) { reply("ERR gfdata_args\n"); return true; }
        n -= n % gs.R;
        uint32_t *hits = heap_caps_malloc(256 * 4, MALLOC_CAP_8BIT); gf_pkt_t *pk = heap_caps_malloc(48 * sizeof(gf_pkt_t), MALLOC_CAP_8BIT);
        uint8_t *pool = heap_caps_malloc(4096, MALLOC_CAP_8BIT), *bits = heap_caps_malloc(GF_MAXBITS, MALLOC_CAP_8BIT), *rec = heap_caps_malloc(48 * 20 + 4096, MALLOC_CAP_8BIT);
        int32_t *sb = heap_caps_malloc(GF_SBUF * 4, MALLOC_CAP_8BIT);
        if (!hits || !pk || !pool || !bits || !rec || !sb) { free(hits); free(pk); free(pool); free(bits); free(rec); free(sb); reply("ERR mem\n"); return true; }
        gs.bits = bits; gs.sbuf = sb;
        prepare_rx(); uint32_t el;
        const char *err = fast_capture(n, div, &el);
        if (!err) {
            unsigned np = gf_block(&gs, DUMP_BUF, n, hits, 256, pk, 48, pool, 4096), o = 0;
            for (unsigned j = 0; j < np; j++) {
                const gf_pkt_t *p = &pk[j];
                memcpy(rec + o, &p->pos, 4); memcpy(rec + o + 4, &p->n, 2); rec[o + 6] = p->flags; rec[o + 7] = p->errs;
                memcpy(rec + o + 8, &p->rssi10, 2); memcpy(rec + o + 10, &p->cfo_hz, 4); memcpy(rec + o + 14, &p->dev_hz, 4); o += 18;
                memcpy(rec + o, pool + p->off, p->n); o += p->n;
            }
            opt_pack(DUMP_BUF, n, 20);
            if (opt_send_frame(DUMP_BUF, opt_wire(n, 20), n, el, true)) {
                char h[96]; snprintf(h, sizeof(h), "GFX %u %u %08" PRIx32 " %" PRIu32 "\n", np, o, esp_rom_crc32_le(0, rec, o), gs.hits);
                (void)(send_bytes(h, strlen(h)) && send_bytes(rec, o));
            }
        } else reply(err);
        free(hits); free(pk); free(pool); free(bits); free(rec); free(sb); return true;
    }
    /* Test-vector path (diagnostics): DMLOAD <word_off> <hex20> ... fills an 8192-word heap buffer
     * (5 hex digits per word, I bits 0-9, Q bits 10-19); DMVFM / DMGF run one block of the DSP on it with
     * persistent state (reset 1 = new run) and return the records plus the CPU cycles of the call. */
    if (!strncmp(line, "DMLOAD ", 7)) {
        unsigned off; int nn = 0;
        if (sscanf(line + 7, "%u%n", &off, &nn) != 1 || off >= 8192) { reply("ERR dmload_args\n"); return true; }
        if (!dm_up && !(dm_up = heap_caps_malloc(8192 * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT))) { reply("ERR mem\n"); return true; }
        const char *p = line + 7 + nn; unsigned k = 0;
        while (*p && off + k < 8192) {
            while (*p == ' ') p++;
            if (!*p) break;
            char *e; unsigned long x = strtoul(p, &e, 16);
            if (e == p) { reply("ERR dmload_hex\n"); return true; }
            dm_up[off + k++] = (uint32_t)x; p = e;
        }
        char h[32]; snprintf(h, sizeof(h), "OK %u\n", k); reply(h); return true;
    }
    if (!strncmp(line, "DMVFM ", 6)) {
        unsigned div, rst; uint32_t gap; int64_t t0; int nn = 0;
        if (!dm_up || sscanf(line + 6, "%u %u %" SCNu32 " %" SCNd64 " %u%n", &div, &n, &gap, &t0, &rst, &nn) != 5 || (div != 2 && div != 3) || n > 8192 || n < 512) { reply("ERR dmvfm_args\n"); return true; }
        if (rst) {
            if (!vf_parse(line + 6 + nn, &dm_vs)) { reply("ERR dmvfm_args\n"); return true; }
            dm_vs.fin = hw_rates[div];
            if (!vf_init(&dm_vs)) { reply("ERR dmvfm_args\n"); return true; }
            free(dm_bins); free(dm_runs);
            free(dm_tab); dm_tab = NULL;
            if (dm_pol & 1) { dm_tab = heap_caps_malloc(4096, MALLOC_CAP_DMA | MALLOC_CAP_8BIT); if (dm_tab) memcpy(dm_tab, vf_atan_tab, 4096); }
            dm_bins = (dm_pol & 2) ? heap_caps_malloc((8192 / dm_vs.B + 4) * 2, MALLOC_CAP_RTCRAM) : dm_fast_alloc((8192 / dm_vs.B + 4) * 2);
            vf_bins_lp = (uintptr_t)dm_bins >= 0x50000000u; dm_runs = heap_caps_malloc(384 * sizeof(vf_run_t), MALLOC_CAP_8BIT);
            dm_vs.tab = dm_tab ? dm_tab : vf_atan_tab; dm_vs.bins = dm_bins; dm_vs.runs = dm_runs; dm_vs.maxruns = 384;
        }
        if (!dm_bins || !dm_runs || !dm_vs.fin) { reply("ERR dmvfm_state\n"); return true; }
        unsigned maxl = n / (dm_vs.B * 300) + 2; uint8_t *out = dm_fast_alloc(maxl * (VF_REC + dm_vs.px));
        if (!out) { reply("ERR mem\n"); return true; }
        uint32_t c0 = esp_cpu_get_cycle_count();
        unsigned nl = vf_block(&dm_vs, dm_up, n - n % dm_vs.B, gap, t0, out, maxl), bytes = nl * (VF_REC + dm_vs.px);
        uint32_t cy = esp_cpu_get_cycle_count() - c0;
        vf_last = dm_vs; vf_have = true;                      /* DSTAT? shows the test-vector run */
        char h[128]; snprintf(h, sizeof(h), "DMV %u %u %08" PRIx32 " %" PRIu32 " %d %u\n", nl, dm_vs.px, esp_rom_crc32_le(0, out, bytes), cy, dm_vs.locked, dm_vs.vlocked);
        (void)(send_bytes(h, strlen(h)) && send_bytes(out, bytes));
        free(out); return true;
    }
    if (!strncmp(line, "DMGF ", 5)) {
        unsigned div, rst; int nn = 0;
        if (!dm_up || sscanf(line + 5, "%u %u %u%n", &div, &n, &rst, &nn) != 3 || div > 5 || n > 8192 || n < 512) { reply("ERR dmgf_args\n"); return true; }
        if (rst) {
            if (!gf_parse(line + 5 + nn, div, &dm_gs)) { reply("ERR dmgf_args\n"); return true; }
            dm_gs.fin = hw_rates[div];
            if (!gf_init(&dm_gs)) { reply("ERR dmgf_args\n"); return true; }
        }
        if (!dm_gs.fin) { reply("ERR dmgf_state\n"); return true; }
        uint32_t *hits = heap_caps_malloc(256 * 4, MALLOC_CAP_8BIT); gf_pkt_t *pk = heap_caps_malloc(48 * sizeof(gf_pkt_t), MALLOC_CAP_8BIT);
        uint8_t *pool = heap_caps_malloc(4096, MALLOC_CAP_8BIT), *bits = heap_caps_malloc(GF_MAXBITS, MALLOC_CAP_8BIT), *rec = heap_caps_malloc(48 * 18 + 4096, MALLOC_CAP_8BIT);
        int32_t *sb = heap_caps_malloc(GF_SBUF * 4, MALLOC_CAP_8BIT);
        if (!hits || !pk || !pool || !bits || !rec || !sb) { free(hits); free(pk); free(pool); free(bits); free(rec); free(sb); reply("ERR mem\n"); return true; }
        dm_gs.bits = bits; dm_gs.sbuf = sb;
        uint32_t c0 = esp_cpu_get_cycle_count();
        unsigned np = gf_block(&dm_gs, dm_up, n - n % dm_gs.R, hits, 256, pk, 48, pool, 4096), o = 0;
        uint32_t cy = esp_cpu_get_cycle_count() - c0;
        for (unsigned j = 0; j < np; j++) {
            const gf_pkt_t *p = &pk[j];
            memcpy(rec + o, &p->pos, 4); memcpy(rec + o + 4, &p->n, 2); rec[o + 6] = p->flags; rec[o + 7] = p->errs;
            memcpy(rec + o + 8, &p->rssi10, 2); memcpy(rec + o + 10, &p->cfo_hz, 4); memcpy(rec + o + 14, &p->dev_hz, 4); o += 18;
            memcpy(rec + o, pool + p->off, p->n); o += p->n;
        }
        char h[96]; snprintf(h, sizeof(h), "DMG %u %u %08" PRIx32 " %" PRIu32 "\n", np, o, esp_rom_crc32_le(0, rec, o), cy);
        (void)(send_bytes(h, strlen(h)) && send_bytes(rec, o));
        dm_gs.bits = NULL; dm_gs.sbuf = NULL; gf_last = dm_gs; gf_have = true;
        free(hits); free(pk); free(pool); free(bits); free(rec); free(sb); return true;
    }
    /* DMBENCH: cycles per input sample of the demodulator hot loops on a heap copy of the last dump */
    if (!strcmp(line, "DMBENCH")) {
        uint32_t *w = heap_caps_malloc(8192 * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); int16_t *v = heap_caps_malloc(4100 * 2, MALLOC_CAP_8BIT);
        vf_run_t *runs = heap_caps_malloc(384 * sizeof(vf_run_t), MALLOC_CAP_8BIT); uint8_t *tab = heap_caps_malloc(4096, MALLOC_CAP_8BIT);
        uint32_t *hits = heap_caps_malloc(256 * 4, MALLOC_CAP_8BIT);
        if (!w || !v || !runs || !tab || !hits) { free(w); free(v); free(runs); free(tab); free(hits); reply("ERR mem\n"); return true; }
        memcpy(w, DUMP_BUF, 8192 * 4); memcpy(tab, vf_atan_tab, 4096);
        uint32_t cy[12], c0;
#define DMB(i, expr) do { c0 = esp_cpu_get_cycle_count(); expr; cy[i] = esp_cpu_get_cycle_count() - c0; } while (0)
        DMB(0, vf_disc_41(w, 2048, tab, 3, v)); DMB(1, vf_disc_42(w, 2048, tab, 3, v)); DMB(2, vf_disc_44(w, 2048, tab, 3, v));
        DMB(3, vf_disc_21(w, 4096, tab, 3, v)); DMB(4, vf_disc_22(w, 4096, tab, 3, v));
        vf_disc_42(w, 2048, tab, 3, v);
        DMB(5, vf_find_runs(v, 2048, 1, 0, runs, 384));
        static gf_t gb; memset(&gb, 0, sizeof(gb)); gb.R = 1; gb.sps = 4; gb.mode = 1; gb.len = 8; gb.sync = 0x12345678u; gb.nbits = 32; gf_init(&gb);
        uint32_t ov = 0;
        DMB(6, gf_scan_1_4(&gb, w, 8191, 0, 0, hits, 256, &ov)); DMB(7, gf_scan_2_4(&gb, w, 4095, 0, 0, hits, 256, &ov));
        DMB(8, gf_scan_2_2(&gb, w, 4095, 0, 0, hits, 256, &ov)); DMB(9, gf_scan_4_8(&gb, w, 2047, 0, 0, hits, 256, &ov));
        DMB(10, gf_scan_1_8(&gb, w, 8191, 0, 0, hits, 256, &ov)); DMB(11, gf_scan_1_2(&gb, w, 8191, 0, 0, hits, 256, &ov));
#undef DMB
        char h[300];
        snprintf(h, sizeof(h), "DMBENCH vfd41 %.2f vfd42 %.2f vfd44 %.2f vfd21 %.2f vfd22 %.2f runs20 %.2f gf14 %.2f gf24 %.2f gf22 %.2f gf48 %.2f gf18 %.2f gf12 %.2f\n",
                 cy[0] / 8192.0, cy[1] / 8192.0, cy[2] / 8192.0, cy[3] / 8192.0, cy[4] / 8192.0, cy[5] / 8192.0, cy[6] / 8192.0, cy[7] / 8192.0, cy[8] / 8192.0,
                 cy[9] / 8192.0, cy[10] / 8192.0, cy[11] / 8192.0);
        free(w); free(v); free(runs); free(tab); free(hits); reply(h); return true;
    }
    return false;
}
