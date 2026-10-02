/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
 *
 * Host harness for main/families/c5_c6_c61/demod_dsp.h (bit-exact with the device).
 * Reads raw dump words (u32 LE: I bits 0-9, Q bits 10-19, device orientation),
 * cuts them into engine blocks of <words> with <gap> samples dropped between
 * blocks (like the continuous engine), runs VFM or GFSK and writes the records.
 *
 *   demod_host vfm  in.u32 out.bin fin words gap D px std pol sel rate_Bps
 *   demod_host gfsk in.u32 out.bin fin words gap R sps mode len sync nbits maxerr chan okonly
 *   demod_host vfm1 / gfsk1 ...   same, but one single block (= VFMDATA / GFDATA on the device)
 *   demod_host vfmp in.u8 out.bin 0 bytes gap D px std pol sel rate_Bps [cseg every step col.bin]
 *                                                     PARLIO bytes at 40 MS/s (vf_block_p8), optional colour records
 *                                                     (VFMCOLOR: per record [u32 block][VC_REC + 2 * cseg]) to col.bin
 *
 * VFM output: per line [u32 block][VF_REC + px record].
 * GFSK output: per packet [u32 block][u32 pos][u16 n][u8 flags][u8 errs][i16 rssi10][i32 cfo][i32 dev] + n bytes.
 * Build: cc -O2 -o demod_host demod_host.c -I../../main/families/c5_c6_c61
 * (tests/test_espsoup_c5.py builds and runs it together with the node scripts).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include "demod_dsp.h"

static uint32_t *load(const char *fn, size_t *n) {
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); exit(1); }
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    uint32_t *w = malloc(sz); if (fread(w, 1, sz, f) != (size_t)sz) { perror("read"); exit(1); }
    fclose(f); *n = sz / 4; return w;
}

int main(int argc, char **argv) {
    if (argc < 7) { fprintf(stderr, "usage: see source\n"); return 2; }
    const char *mode = argv[1]; size_t nw; uint32_t *w = load(argv[2], &nw);
    FILE *fo = fopen(argv[3], "wb");
    unsigned fin = atoi(argv[4]), words = atoi(argv[5]), gap = atoi(argv[6]);
    int single = mode[strlen(mode) - 1] == '1';
    if (single) { words = (unsigned)nw; gap = 0; }
    clock_t t0 = clock();
    if (!strcmp(mode, "vfmp")) {          /* PARLIO bytes (40 MS/s, low nibble I, high nibble Q): vf_block_p8 */
        const uint8_t *x = (const uint8_t *)w; size_t nx = nw * 4;
        vf_t s; memset(&s, 0, sizeof(s));
        s.fin = 20000000; s.D = atoi(argv[7]); s.px = atoi(argv[8]); s.std_force = atoi(argv[9]); s.pol_force = atoi(argv[10]);
        s.sel = atoi(argv[11]); s.rate_Bps = atoi(argv[12]); s.src = 1;
        if (!vf_init(&s)) { fprintf(stderr, "bad vfm config\n"); return 2; }
        s.tab = vf_ptab; s.bins = malloc(sizeof(int16_t) * (VFP_KEEP + words / 8 + 4)); s.maxruns = 1024; s.runs = malloc(sizeof(vf_run_t) * s.maxruns);
        unsigned maxl = (VFP_KEEP + words / 8) / 300 + 4; uint8_t *out = malloc(maxl * (VF_REC + s.px));
        static vc_t vc; FILE *fc = NULL; unsigned crec = 0;
        if (argc > 16 && atoi(argv[13]) > 0) {
            memset(&vc, 0, sizeof(vc)); vc.nseg = atoi(argv[13]); vc.every = atoi(argv[14]); vc.step = atoi(argv[15]);
            if (!vc_init(&vc)) { fprintf(stderr, "bad colour config\n"); return 2; }
            crec = VC_REC + 2 * vc.nseg; vc.max = maxl; vc.out = malloc(maxl * crec); s.vc = &vc;
            fc = fopen(argv[16], "wb");
        }
        uint32_t blk = 0; size_t pos = 0; unsigned g = 0;
        while (pos + words <= nx) {
            int64_t t16 = (int64_t)pos * 2 / 5;                 /* 40 MS/s: 2/5 of 1/16 us per byte */
            if (s.vc) vc.n = 0;
            unsigned nl = vf_block_p8(&s, x + pos, words, g, t16, out, maxl);
            for (unsigned j = 0; j < nl; j++) { fwrite(&blk, 4, 1, fo); fwrite(out + j * (VF_REC + s.px), 1, VF_REC + s.px, fo); }
            if (fc) for (unsigned j = 0; j < vc.n; j++) { fwrite(&blk, 4, 1, fc); fwrite(vc.out + j * crec, 1, crec, fc); }
            pos += words + gap; g = gap; blk++;
        }
        printf("VFM blocks %u lines %u sync %u fly %u out %u drop_rate %u drop_cut %u vbi %u vsyncs %u relocks %u acq %u pol %d std %d qs %u tip16 %d blank16 %d T16 %d\n",
               s.blocks, s.lines, s.l_sync, s.l_fly, s.l_out, s.l_drop_rate, s.l_drop_cut, s.l_vbi, s.vsyncs, s.relocks, s.acq, s.pol, s.std, s.qs, s.tip16, s.blank16, s.T16);
        if (fc) {
            printf("VFC lines %u ref2 %u weak %u drop_rate %u drop_full %u drop_cut %u pre %u P %u G %u segb %u sgr %u cyc %llu\n", vc.lines, vc.ref2, vc.weak,
                   vc.drop_rate, vc.drop_full, vc.drop_cut, vc.pre, vc.P, vc.G, vc.segb, vc.sgr, (unsigned long long)s.cyc_color);
            fclose(fc);
        }
    } else if (!strncmp(mode, "vfm", 3)) {
        vf_t s; memset(&s, 0, sizeof(s));
        s.fin = fin; s.D = atoi(argv[7]); s.px = atoi(argv[8]); s.std_force = atoi(argv[9]); s.pol_force = atoi(argv[10]);
        s.sel = atoi(argv[11]); s.rate_Bps = atoi(argv[12]);
        if (!vf_init(&s)) { fprintf(stderr, "bad vfm config\n"); return 2; }
        s.tab = vf_atan_tab; s.bins = malloc(sizeof(int16_t) * (words / s.B + 4)); s.maxruns = 1024; s.runs = malloc(sizeof(vf_run_t) * s.maxruns);
        unsigned maxl = words / (s.B * 300) + 4; uint8_t *out = malloc(maxl * (VF_REC + s.px));
        uint32_t blk = 0; size_t pos = 0; unsigned g = 0;
        while (pos + words <= nw) {
            int64_t t_us = (int64_t)pos * 1000000 / fin;
            unsigned nl = vf_block(&s, w + pos, words, g, t_us, out, maxl);
            for (unsigned j = 0; j < nl; j++) { fwrite(&blk, 4, 1, fo); fwrite(out + j * (VF_REC + s.px), 1, VF_REC + s.px, fo); }
            pos += words + gap; g = gap; blk++;
        }
        printf("VFM blocks %u lines %u sync %u fly %u out %u drop_rate %u drop_cut %u vbi %u vsyncs %u relocks %u acq %u pol %d std %d qs %u tip16 %d blank16 %d T16 %d\n",
               s.blocks, s.lines, s.l_sync, s.l_fly, s.l_out, s.l_drop_rate, s.l_drop_cut, s.l_vbi, s.vsyncs, s.relocks, s.acq, s.pol, s.std, s.qs, s.tip16, s.blank16, s.T16);
    } else {
        gf_t g; memset(&g, 0, sizeof(g));
        g.fin = fin; g.R = atoi(argv[7]); g.sps = atoi(argv[8]); g.mode = atoi(argv[9]); g.len = atoi(argv[10]);
        g.sync = (uint32_t)strtoul(argv[11], NULL, 16); g.nbits = atoi(argv[12]); g.maxerr = atoi(argv[13]); g.chan = atoi(argv[14]); g.okonly = atoi(argv[15]);
        if (!gf_init(&g)) { fprintf(stderr, "bad gfsk config\n"); return 2; }
        uint8_t bits[GF_MAXBITS]; static int32_t sbuf[GF_SBUF]; g.bits = bits; g.sbuf = sbuf;
        uint32_t hits[256]; gf_pkt_t pk[64]; static uint8_t pool[8192];
        uint32_t blk = 0; size_t pos = 0;
        while (pos + words <= nw) {
            unsigned np = gf_block(&g, w + pos, words, hits, 256, pk, 64, pool, sizeof(pool));
            for (unsigned j = 0; j < np; j++) {
                uint32_t p = (uint32_t)pos + pk[j].pos;
                fwrite(&blk, 4, 1, fo); fwrite(&p, 4, 1, fo); fwrite(&pk[j].n, 2, 1, fo); fwrite(&pk[j].flags, 1, 1, fo); fwrite(&pk[j].errs, 1, 1, fo);
                fwrite(&pk[j].rssi10, 2, 1, fo); fwrite(&pk[j].cfo_hz, 4, 1, fo); fwrite(&pk[j].dev_hz, 4, 1, fo); fwrite(pool + pk[j].off, 1, pk[j].n, fo);
            }
            pos += words + gap; blk++;
        }
        printf("GFSK blocks %u hits %u hit_ovf %u pkts %u crc_ok %u trunc %u rejected %u pool_ovf %u\n",
               g.blocks, g.hits, g.hit_ovf, g.pkts, g.crc_ok, g.trunc, g.rejected, g.pool_ovf);
    }
    fprintf(stderr, "host time %.1f ms for %zu samples\n", (clock() - t0) * 1000.0 / CLOCKS_PER_SEC, nw);
    fclose(fo);
    return 0;
}
