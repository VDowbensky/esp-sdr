/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 ESP-SDR contributors (ESPARGOS) - original algorithm
 * Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser - ESP32-C5 port
 *
 * Hardware DC correction of the baseband DC DACs for the ESP32-C5 (ESPsoup).
 *
 * Source: port of receiver_dc_calibrate() from main/targets/esp32s31/streaming/dc.c
 * of this repository (ESPARGOS ESP-SDR, GPL-3.0-or-later): measure the DC, measure
 * the 2x2 response of the two DC DACs (+-16 codes each), Newton steps with step
 * halving, latch the best measured codes; nothing is stored permanently.
 * C5 adaptation: the same PBUS fields (the C5 libphy phy_pbus_set_dco writes
 * block 2/3, bank 1/2; as on the S31, bank 2 is adjusted), the DC is measured
 * from a 20 MS/s dump snapshot (10 bit, 16380 samples) instead of the S31
 * PARLIO bytes; thresholds in ADC8 counts as upstream (10-bit mean / 4).
 *
 * The correction lives in PBUS debug mode: the next prepare_rx() (FREQ, BANDWIDTH, LPF, TUNEM ...) switches back
 * to work mode and the table values, i.e. DCCAL must be repeated after every retune (DCAUTO does it for the
 * continuous runs).  Use a MANUAL gain: in debug mode the PBUS no longer follows the hardware AGC.
 */
#pragma once
extern void phy_pbus_debugmode(void);
extern void phy_pbus_force_test(unsigned, unsigned, unsigned);
extern unsigned phy_pbus_rd(unsigned, unsigned);

static float dc_before[2], dc_after[2], dc_jac[4];
static unsigned dc_steps, dc_runs; static int dc_best[2] = {-1, -1}, dc_base[2] = {-1, -1};
static bool dc_auto;              /* DCAUTO 1: DCCAL before VFM / STREAM / GFSK / CSPEC runs */
/* The PBUS stays in debug mode with forced values after a calibration; it ignores later gain changes
 * until it is back in work mode (measured: a GAIN change after a PARLIO run left the old gain active). */
static bool dc_forced;

/* mean I, Q of a 20 MS/s snapshot (device orientation) in ADC8 counts */
static bool dc_measure(float m[2]) {
    uint32_t el;
    if (fast_capture(16380, 2, &el)) return false;
    int64_t si = 0, sq = 0;
    for (unsigned j = 0; j < 16380; j++) { uint32_t w = DUMP_BUF[j]; si += ((int32_t)(w << 22)) >> 22; sq += ((int32_t)(w << 12)) >> 22; }
    m[0] = (float)si / (16380.0f * 4.0f); m[1] = (float)sq / (16380.0f * 4.0f);
    return true;
}
static int dc_clamp(int v) { return v < 0 ? 0 : v > 511 ? 511 : v; }
static void dc_apply(int i, int q) {
    phy_pbus_force_test(2, 2, (unsigned)i);
    phy_pbus_force_test(3, 2, (unsigned)q);
    vTaskDelay(pdMS_TO_TICKS(10));
}
static float dc_cost(const float m[2]) { return m[0] * m[0] + m[1] * m[1]; }

static void dc_calibrate(void) {
    prepare_rx();
    if (gain_mode == GAIN_HARDWARE) { dc_steps = 0; dc_runs++; dc_before[0] = dc_before[1] = dc_after[0] = dc_after[1] = -1; return; }   /* AGC needs work mode */
    dc_steps = 0; dc_runs++;
    memset(dc_jac, 0, sizeof(dc_jac)); memset(dc_before, 0, sizeof(dc_before)); memset(dc_after, 0, sizeof(dc_after));
    dc_best[0] = dc_best[1] = dc_base[0] = dc_base[1] = -1;
    if (!dc_measure(dc_before)) return;
    memcpy(dc_after, dc_before, sizeof(dc_after));
    if (dc_cost(dc_before) < .25f) return;
    unsigned live[4][2];
    for (unsigned block = 0; block < 4; block++)
        for (unsigned bank = 1; bank <= 2; bank++) live[block][bank - 1] = phy_pbus_rd(block, bank);
    int base[2] = {(int)live[2][1], (int)live[3][1]};
    dc_base[0] = base[0]; dc_base[1] = base[1];
    /* keep the live RF/baseband state while taking over the PBUS (work mode re-asserts the table values) */
    phy_pbus_debugmode();
    dc_forced = true;
    for (unsigned block = 0; block < 4; block++)
        for (unsigned bank = 1; bank <= 2; bank++) phy_pbus_force_test(block, bank, live[block][bank - 1]);
    int best[2] = {base[0], base[1]};
    float best_cost = dc_cost(dc_before);
    int di = base[0] > 495 ? -16 : 16, dq = base[1] > 495 ? -16 : 16;
    float mi[2], mq[2], obs[2];
    dc_apply(base[0] + di, base[1]);
    if (!dc_measure(mi)) { dc_apply(base[0], base[1]); return; }
    dc_apply(base[0], base[1] + dq);
    if (!dc_measure(mq)) { dc_apply(base[0], base[1]); return; }
    float a = (mi[0] - dc_before[0]) / di, b = (mq[0] - dc_before[0]) / dq;
    float c = (mi[1] - dc_before[1]) / di, d = (mq[1] - dc_before[1]) / dq;
    dc_jac[0] = a; dc_jac[1] = b; dc_jac[2] = c; dc_jac[3] = d;
    float det = a * d - b * c;
    if (fabsf(det) < .002f || fabsf(a) > 4 || fabsf(b) > 4 || fabsf(c) > 4 || fabsf(d) > 4) { dc_apply(base[0], base[1]); return; }
    for (unsigned step = 0; step < 5 && best_cost > .25f; step++) {
        float delta[2] = {(-d * dc_after[0] + b * dc_after[1]) / det, (c * dc_after[0] - a * dc_after[1]) / det};
        bool improved = false;
        for (unsigned trial = 0; trial < 3; trial++) {
            int next[2];
            for (unsigned axis = 0; axis < 2; axis++) {
                float change = fmaxf(-32.f, fminf(32.f, delta[axis])) / (float)(1u << trial);
                next[axis] = dc_clamp(best[axis] + (int)roundf(change));
                if (next[axis] < base[axis] - 96) next[axis] = base[axis] - 96;
                if (next[axis] > base[axis] + 96) next[axis] = base[axis] + 96;
            }
            if (next[0] == best[0] && next[1] == best[1]) break;
            dc_apply(next[0], next[1]);
            dc_steps++;
            if (dc_measure(obs) && dc_cost(obs) < best_cost) {
                best_cost = dc_cost(obs); best[0] = next[0]; best[1] = next[1];
                memcpy(dc_after, obs, sizeof(obs)); improved = true; break;
            }
        }
        if (!improved) break;
    }
    dc_apply(best[0], best[1]);      /* the last trial may be worse: latch the best measured codes */
    dc_best[0] = best[0]; dc_best[1] = best[1];
}
static void dc_report(void) {
    char h[200];
    snprintf(h, sizeof(h), "DCCAL before %.2f %.2f after %.2f %.2f base %d %d best %d %d steps %u jac %.3f %.3f %.3f %.3f auto %d\n", dc_before[0], dc_before[1], dc_after[0],
             dc_after[1], dc_base[0], dc_base[1], dc_best[0], dc_best[1], dc_steps, dc_jac[0], dc_jac[1], dc_jac[2], dc_jac[3], (int)dc_auto);
    reply(h);
}

/* DCCAL: calibrate now and report.  DCCAL? reports the last result ("DC?" is the
 * upstream spectrum DC-tracker query, see spectrum.c).  DCAUTO 0/1: calibrate
 * before every VFM / STREAM / GFSK / CSPEC run. */
static bool dccal_command(const char *line) {
    unsigned n; char extra;
    if (!strcmp(line, "DCCAL")) { dc_calibrate(); dc_report(); return true; }
    if (!strcmp(line, "DCCAL?")) { dc_report(); return true; }
    if (sscanf(line, "DCAUTO %u %c", &n, &extra) == 1 && n <= 1) { dc_auto = n; reply("OK\n"); return true; }
    /* a gain change drops the correction: PBUS back to work mode, then gain_command applies the gain */
    if (dc_forced && !strncmp(line, "GAIN ", 5)) { phy_pbus_workmode(); dc_forced = false; }
    if (dc_auto && (!strncmp(line, "VFM ", 4) || !strncmp(line, "STREAM", 6) || !strncmp(line, "GFSK ", 5) || !strncmp(line, "CSPEC ", 6))) dc_calibrate();
    return false;
}
