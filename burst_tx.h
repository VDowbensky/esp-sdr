#pragma once
/* ISM-only test transmitter using the PHY certification API (C5 only).
 * Included by the receiver after prepare_rx(); relies on rx_ready, reply()
 * and phy_chip_set_chan (the C5 exact-MHz tuning helper).
 *
 * Commands:
 *   TXTONE <MHz> <backoff> <ms>  continuous carrier
 *   TXPKT  <MHz> <backoff> <ms>  802.11b 1 Mbit/s test frames (channel centres only)
 *   TXOFF                        stop transmitting
 *   TX?                          TX OFF | TX TONE|PKT <MHz> <backoff> <remaining ms>
 * backoff is in 0.25 dB below maximum output power. Transmission always
 * stops after <ms> (at most TX_MAX_MS), and before any receive command. */
#include "esp_phy_cert_test.h"

#define TX_MAX_MS 30000u
#define TX_MAX_BACKOFF 80u      /* -20 dB */
#define TX_5G_MIN_BACKOFF 24u   /* -6 dB: keep 5.8 GHz SRD output near 25 mW */

enum { TX_OFF, TX_TONE, TX_PKT };
static int tx_mode = TX_OFF;
static unsigned tx_mhz, tx_chan, tx_backoff;
static int64_t tx_deadline;

#ifdef TX_LAB_UNLOCK
/* Shielded-lab build: any tunable MHz, full power. Never ship this build. */
#undef TX_5G_MIN_BACKOFF
#define TX_5G_MIN_BACKOFF 0u
static bool tx_band_ok(unsigned mhz) { return rx_frequency_valid(mhz); }
#else
static bool tx_band_ok(unsigned mhz) {
    return (mhz >= 2402 && mhz <= 2480) || (mhz >= 5730 && mhz <= 5870);
}
#endif
static unsigned tx_channel(unsigned mhz) {
    if (mhz < 3000) {
        int ch = ((int)mhz - 2407 + 2) / 5;
        return ch < 1 ? 1 : ch > 13 ? 13 : (unsigned)ch;
    }
    int ch = ((int)mhz - 5000 + 2) / 5;
    return ch < 36 ? 36 : ch > 177 ? 177 : (unsigned)ch;
}
static unsigned tx_channel_mhz(unsigned ch) { return ch < 15 ? 2407 + 5 * ch : 5000 + 5 * ch; }

static void tx_stop(void) {
    if (tx_mode == TX_TONE) esp_phy_wifi_tx_tone(0, tx_chan, 0);
    else if (tx_mode == TX_PKT) esp_phy_test_start_stop(0);
    if (tx_mode != TX_OFF) { tx_mode = TX_OFF; rx_ready = false; }
}
static void tx_poll(void) {
    if (tx_mode != TX_OFF && esp_timer_get_time() >= tx_deadline) tx_stop();
}

/* Spectrum painting. The carrier is moved by writing the 24-bit PLL
 * sigma-delta word (analog block 0x63, registers 3-5) directly: about 3 us
 * per register instead of ~400 us through phy_set_rf_freq_offset. The word
 * scale is measured at start from a 1 MHz offset, so it holds for either band. */
#define PAINT_MAX_COLS 160
static struct {
    unsigned mhz, cols, step_khz, backoff;
    bool ready;
    int32_t w0, lsb_per_mhz;
    uint32_t words[PAINT_MAX_COLS];
    uint8_t last[3];
} paint;

static uint32_t pll_word_read(void) {
    return (phy_chip_i2c_readReg(0x63, 0, 3) & 255) << 16 | (phy_chip_i2c_readReg(0x63, 0, 4) & 255) << 8 |
           (phy_chip_i2c_readReg(0x63, 0, 5) & 255);
}
static void pll_word_write(uint32_t w) {
    for (unsigned j = 0; j < 3; j++) {
        uint8_t b = (w >> (16 - 8 * j)) & 255;
        if (b != paint.last[j]) { phy_i2c_writeReg(0x63, 0, 3 + j, b); paint.last[j] = b; }
    }
}
static bool paint_start(void) {
    tx_stop();
    phy_pbus_xpd_rx_on(0);
    unsigned ch = paint.mhz > 3000 ? 36 : tx_channel(paint.mhz);
    esp_phy_wifi_tx_tone(1, ch, paint.backoff);
    phy_set_rf_freq_offset(phy_param[49], paint.mhz, 1000);
    int32_t w1 = (int32_t)pll_word_read();
    phy_set_rf_freq_offset(phy_param[49], paint.mhz, 0);
    paint.w0 = (int32_t)pll_word_read();
    paint.lsb_per_mhz = ((w1 - paint.w0) << 8) >> 8;  /* signed 24-bit difference */
    if (paint.lsb_per_mhz < 1000) { esp_phy_wifi_tx_tone(0, ch, 0); return false; }
    for (unsigned i = 0; i < paint.cols; i++) {
        int64_t d = (int64_t)(2 * (int)i - (int)paint.cols + 1) * paint.lsb_per_mhz * paint.step_khz / 2000;
        paint.words[i] = (uint32_t)(paint.w0 + d) & 0xffffff;
    }
    for (unsigned j = 0; j < 3; j++) paint.last[j] = (paint.w0 >> (16 - 8 * j)) & 255;
    tx_mode = TX_TONE; tx_mhz = paint.mhz; tx_chan = ch; tx_backoff = paint.backoff; rx_ready = false;
    tx_deadline = esp_timer_get_time() + 5000000;
    paint.ready = true;
    return true;
}
static void paint_row(const char *hex, unsigned ms) {
    uint16_t on[PAINT_MAX_COLS]; unsigned n = 0;
    for (unsigned i = 0; i < paint.cols && hex[i / 4]; i++) {
        char c = hex[i / 4];
        unsigned v = c >= '0' && c <= '9' ? c - '0' : (c | 32) >= 'a' && (c | 32) <= 'f' ? (c | 32) - 'a' + 10 : 0;
        if (v & (8 >> (i % 4))) on[n++] = i;
    }
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000, yield = esp_timer_get_time() + 20000;
    if (!n) {  /* blank row: carrier off */
        if (tx_mode != TX_OFF) { esp_phy_wifi_tx_tone(0, tx_chan, 0); tx_mode = TX_OFF; paint.ready = false; }
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }
    if (!paint.ready || tx_mode != TX_TONE) paint_start();
    while (esp_timer_get_time() < end) {
        for (unsigned k = 0; k < n; k++) pll_word_write(paint.words[on[k]]);
        if (esp_timer_get_time() > yield) { vTaskDelay(1); yield = esp_timer_get_time() + 20000; }
    }
    tx_deadline = esp_timer_get_time() + 5000000;  /* keep the carrier between rows */
}

static bool tx_command(const char *line) {
    unsigned mhz, backoff, ms; char extra;
    if (!strcmp(line, "TXOFF")) { tx_stop(); reply("OK\n"); return true; }
    if (!strcmp(line, "TX?")) {
        char h[80];
        if (tx_mode == TX_OFF) snprintf(h, sizeof(h), "TX OFF\n");
        else snprintf(h, sizeof(h), "TX %s %u %u %u\n", tx_mode == TX_TONE ? "TONE" : "PKT", tx_mhz, tx_backoff,
                      (unsigned)((tx_deadline - esp_timer_get_time()) / 1000));
        reply(h); return true;
    }
    /* TXOFS <MHz> <offset>: retune a running carrier through the PLL offset
     * argument (unit under investigation); replies with the call time. */
    int ofs;
    if (sscanf(line, "TXOFS %u %d %c", &mhz, &ofs, &extra) == 2) {
        if (tx_mode != TX_TONE || !tx_band_ok(mhz)) { reply("ERR tx_state\n"); return true; }
        int64_t t0 = esp_timer_get_time();
        phy_set_rf_freq_offset(phy_param[49], mhz, ofs);
        char h[48]; snprintf(h, sizeof(h), "OK %u\n", (unsigned)(esp_timer_get_time() - t0)); reply(h);
        return true;
    }
    /* Spectrum painting: TXPAINT sets a pixel grid around a centre carrier,
     * TXROW hops the carrier over the set pixels of one image row. */
    unsigned cols, step_khz;
    if (sscanf(line, "TXPAINT %u %u %u %u %c", &mhz, &step_khz, &cols, &backoff, &extra) == 4) {
        if (cols < 1 || cols > PAINT_MAX_COLS || step_khz < 10 || step_khz > 2000) { reply("ERR paint_args\n"); return true; }
        unsigned half = (cols * step_khz / 2 + 999) / 1000;
        if (!tx_band_ok(mhz - half) || !tx_band_ok(mhz + half)) { reply("ERR tx_band\n"); return true; }
        if (backoff > TX_MAX_BACKOFF || (mhz > 3000 && (int)backoff < (int)TX_5G_MIN_BACKOFF)) { reply("ERR tx_power\n"); return true; }
        paint.mhz = mhz; paint.cols = cols; paint.step_khz = step_khz; paint.backoff = backoff; paint.ready = false;
        if (!paint_start()) { reply("ERR paint_pll\n"); return true; }
        char h[64]; snprintf(h, sizeof(h), "OK PAINT %06" PRIx32 " %" PRId32 "\n", paint.w0, paint.lsb_per_mhz); reply(h);
        return true;
    }
    if (!strncmp(line, "TXROW ", 6)) {
        char hex[PAINT_MAX_COLS / 4 + 2];
        if (sscanf(line + 6, "%u %40s %c", &ms, hex, &extra) != 2 || !paint.cols || ms < 1 || ms > 2000) { reply("ERR paint_row\n"); return true; }
        paint_row(hex, ms);
        reply("OK\n");
        return true;
    }
#ifdef TX_LAB_UNLOCK
    /* Lab-only analog register access: I2CD <block> <host> dumps registers 0-31,
     * I2CW <block> <host> <reg> <value> writes one. */
    unsigned blk, host, reg, val;
    if (sscanf(line, "I2CD %x %u %c", &blk, &host, &extra) == 2) {
        char h[200]; int n = snprintf(h, sizeof(h), "I2C");
        for (unsigned r = 0; r < 32; r++) n += snprintf(h + n, sizeof(h) - n, " %02x", phy_chip_i2c_readReg(blk, host, r) & 0xff);
        snprintf(h + n, sizeof(h) - n, "\n"); reply(h); return true;
    }
    if (sscanf(line, "I2CW %x %u %u %x %c", &blk, &host, &reg, &val, &extra) == 4) {
        int64_t t0 = esp_timer_get_time();
        phy_i2c_writeReg(blk, host, reg, val);
        char h[32]; snprintf(h, sizeof(h), "OK %u\n", (unsigned)(esp_timer_get_time() - t0)); reply(h); return true;
    }
#endif
    bool tone = !strncmp(line, "TXTONE ", 7), pkt = !strncmp(line, "TXPKT ", 6);
    if (!tone && !pkt) return false;
    if (sscanf(line + (tone ? 7 : 6), "%u %u %u %c", &mhz, &backoff, &ms, &extra) != 3) { reply("ERR tx_args\n"); return true; }
    if (!tx_band_ok(mhz)) { reply("ERR tx_band\n"); return true; }
    if (backoff > TX_MAX_BACKOFF || (mhz > 3000 && (int)backoff < (int)TX_5G_MIN_BACKOFF)) { reply("ERR tx_power\n"); return true; }
    if (ms < 100 || ms > TX_MAX_MS) { reply("ERR tx_duration\n"); return true; }
    unsigned ch = tx_channel(mhz);
    if (pkt && tx_channel_mhz(ch) != mhz) { reply("ERR tx_channel\n"); return true; }
    tx_stop();
    phy_pbus_xpd_rx_on(0);
    if (tone) {
        /* The PHY tone routine resets the chip on some 5 GHz channels (130 and
         * 140 on C5 rev 1.0), so every 5 GHz carrier starts on channel 36. */
        if (mhz > 3000) ch = 36;
        esp_phy_wifi_tx_tone(1, ch, backoff);
        /* Move the carrier to the exact MHz with the same direct PLL path the
         * receiver uses for arbitrary frequencies. */
        if (tx_channel_mhz(ch) != mhz) phy_set_rf_freq_offset(phy_param[49], mhz, 0);
    } else {
        esp_phy_test_start_stop(3);
        esp_phy_wifi_tx(ch, PHY_RATE_1M, (int8_t)backoff, 1000, 500, 0);
    }
    tx_mode = tone ? TX_TONE : TX_PKT;
    tx_mhz = mhz; tx_chan = ch; tx_backoff = backoff; rx_ready = false;
    tx_deadline = esp_timer_get_time() + (int64_t)ms * 1000;
    char h[64]; snprintf(h, sizeof(h), "OK TX %u %u\n", mhz, ch); reply(h);
    return true;
}
