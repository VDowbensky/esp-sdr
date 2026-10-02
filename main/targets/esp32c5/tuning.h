#pragma once
extern void phy_set_chanfreq(unsigned mhz, unsigned mode);
extern void phy_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);
extern unsigned char phy_param[];
/* ESPsoup: kHz fine-tuning residual (FREQK). The third argument of
 * phy_set_rf_freq_offset is an LO offset in kHz (measured: +100 moves the LO
 * up by 100 kHz). FREQ always resets it to zero. */
static int c5_khz_offset;
/* ESPsoup TUNEM: how the PLL is programmed (measured with a CW generator, 10 MHz steps).
 * 0 = esp-sdr path below (band-calibration channel + phy_set_rf_freq_offset): exact 2130-2730 and
 *     4790-~5745 MHz, about 20 dB less sensitive above ~5745 MHz.
 * 2 = Wi-Fi driver path: esp_wifi_set_channel on a nearby channel, then libphy phy_change_channel(MHz),
 *     as used by esp_wifi based 802.11p/ITS-G5 receivers: exact 4790-5990 MHz incl. UNII-3 and ITS-G5
 *     (5990 MHz = limit of the test generator), same 2.4 GHz behaviour and passband as path 0.
 * 3 = as 2 with the baseband in 802.11p mode (phy_11p_set): worse everywhere, diagnostic only.
 * 4 = auto (default): 2 above 3000 MHz, 0 at 2.4 GHz and whenever a FREQK kHz offset is set
 *     (path 2 has no kHz offset).
 * 5 = experiment (sequence as in the C5VRX project): esp_wifi_set_channel on the nearest public Wi-Fi
 *     channel, then phy_set_freq(MHz, 0).  libphy phy_set_freq ends in phy_chip_set_chan; measured
 *     identical to 4, no range extension.
 * (1 was a PLL bias experiment that locked the LO up to 6.4 GHz but left the receiver deaf; removed.)
 * Neither path tunes below ~2.1 GHz: lower FREQ values receive at a different LO (~2.2 GHz). */
#ifdef ESP_PLATFORM
#include "esp_wifi.h"
#endif
extern void phy_change_channel(int mhz, int a, int b, int c);
extern void phy_11p_set(int on, int b);
extern void phy_set_freq(unsigned mhz, int offset);
static unsigned c5_tune_method = 4;
static bool c5_11p_on;
static bool c5_tune_method_valid(unsigned m) { return m <= 5 && m != 1; }
static void c5_wifi_channel(unsigned mhz, bool nearest) {
    unsigned ch;
    if (mhz > 3000) {
        static const unsigned char ch5[] = {36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
                                      149, 153, 157, 161, 165, 169, 173, 177};
        ch = 140;
        if (nearest) {
            unsigned best = 0, bd = 100000;
            for (unsigned k = 0; k < sizeof(ch5); k++) { unsigned f = 5000 + 5 * ch5[k], d = f > mhz ? f - mhz : mhz - f; if (d < bd) { bd = d; best = k; } }
            ch = ch5[best];
        }
    } else ch = mhz < 2412 ? 1 : mhz > 2472 ? 13 : (mhz - 2407 + 2) / 5;
    (void)esp_wifi_set_channel((unsigned char)ch, WIFI_SECOND_CHAN_NONE);
}
static void c5_set_chan(unsigned mhz, unsigned mode) {
    unsigned m = c5_tune_method;
    if (m == 4) m = mhz > 3000 && !c5_khz_offset ? 2 : 0;
    bool want11p = m == 3;
    if (want11p != c5_11p_on) { phy_11p_set(want11p ? 1 : 0, 0); c5_11p_on = want11p; }
    if (m == 5) {
        c5_wifi_channel(mhz, true);
        phy_set_freq(mhz, 0);
        return;
    }
    if (m == 2 || m == 3) {
        c5_wifi_channel(mhz, false);
        phy_change_channel((int)mhz, 1, 0, 0);
        return;
    }
    /* The pinned PHY selects its 5 GHz path above 3000 MHz. Initialize band
     * calibration on a real channel before programming exact PLL MHz.
     * phy_set_freq goes back through channel conversion on C5; bypass it.
     * phy_param[49] is the crystal selector used by phy_chip_set_chan. */
    phy_set_chanfreq(mhz > 3000 ? 5180 : 2412, mode);
    phy_set_rf_freq_offset(phy_param[49], mhz, c5_khz_offset);
}
#define phy_chip_set_chan c5_set_chan
