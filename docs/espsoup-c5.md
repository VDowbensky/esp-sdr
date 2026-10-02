# ESPsoup extensions for the ESP32-C5

The ESPsoup extensions add receive-only features to the ESP32-C5 burst
receiver of this firmware: a faster register-level capture engine, longer
snapshots, anti-aliased decimation, on-device spectra, a continuous capture
engine (91–96 % of all samples at 4–10 MS/s), tuning up to 5990 MHz through
the Wi-Fi driver channel path, on-device FM-video and GFSK packet
demodulators, a hardware DC-offset calibration and a gap-free 40 MS/s
4+4-bit capture path via the modem diagnostic bus and PARLIO RX.

Everything is **additive**: every command and reply of the upstream protocol
is unchanged, new behaviour is reachable only through new commands that are
announced by new `CAPS` tokens. Clients that do not know the tokens (for
example the upstream browser viewer) see an ordinary C5 burst receiver.
Nothing here enables the transmitter.

Version: `FWOPT?` → `FWOPT 0.7.0 espsoup rx-only C5`. Internal development
versions 0.3–0.6 preceded this first public version; their results are
summarised below.

Contents: [Compatibility](#compatibility) · [Hardware findings](#hardware-findings) ·
[Commands](#commands) · [Measurements](#measurements) ·
[Continuous engine](#continuous-engine) · [Tuning (TUNEM)](#tuning-tunem) ·
[On-device demodulators](#on-device-demodulators) ·
[DC calibration](#hardware-dc-calibration) · [PARLIO path](#gap-free-parlio-path) ·
[Limitations](#limitations) · [Testing](#testing) · [Sources](#sources-and-attribution)

## Compatibility

* `INFO` stays `C5SDR 6 burst 16380`. `CAPS` additionally contains
  `BIGCAP CAPT TEMP FREQK FWOPT DDC SPECDB SWEEP STREAM CONT CSPEC CGRID CPKT TUNEM DEMODFM GFSKPKT DCCAL PARLIO`.
* `LIMITS?` keeps all upstream keys and adds `maxSamples` (32704), `stream`,
  `cspec`, `ddc`, `spec`, `freqStepHz` (1000) and `lockMHz`
  (`[[2130,2730],[4790,5990]]`, the measured exact-tuning ranges).
* `CAP`/`CAP16`/`CAP20`/`RXRUN` accept up to 32704 samples on the C5 (BIGCAP);
  all captures use the register-level engine (`ENGINE STOCK` switches back to
  the vendor `adctrig()` path for comparisons).
* The shared on-chip spectrum protocol (`SPEC <ms> <stride> <units> <det> <rate> <bins> [stats]`,
  `SPECINFO?`, `DC?`/`DC 0|1`, SPC1/SPS1 frames, see [spectrum.md](spectrum.md))
  is unchanged; on the C5 its snapshots come from the fast engine, i.e. without
  the ~1.7 ms vendor stall per FFT window.
* The three-argument `SPEC <rate_idx> <nfft> <navg>` is the older ESPsoup
  averaged spectrum (0.01 dB bins); its CAPS token is `SPECDB`. Six- or
  seven-argument `SPEC` lines always reach the shared protocol.
* Tuning: `RANGE?` reports the upstream `100 6000 1`. `FREQ` resets the
  `FREQK` kHz offset. Above 3000 MHz the default tuning path is the Wi-Fi
  driver path (`TUNEM 4`), see [Tuning](#tuning-tunem).
* The I/Q orientation is the upstream one (the host negates Q); all binary
  payloads carry a CRC32 (zlib polynomial).
* Long-running commands (`STREAM`, `CSPEC`, `CPKT`, `VFM`, `GFSK`, `SWEEP`,
  `PRUN`) stop on any line from the host (e.g. `STOP`) and end with an
  `END …` line.

## Hardware findings

Measured on ESP32-C5 rev 1.0 boards (48 MHz crystal, 4–8 MB flash), native USB.

**Capture path.** `CAP16/CAP20` → `adctrig()` from the vendor `librftest.a`
configures the "RF dump" block: `0x600a9004` CTRL (bits 0–16 count, 31 enable,
19 software trigger, 18 done), `0x600a9008` MODE (bits 21–23 divider, 17–20
source 15, **bits 0–16 = live write position**), `0x600a9018` byte packing,
`0x60095004` SRAM owner (bits 8–11 = 2 → bank 1, bit 16 = half). The dump
writes 32-bit words (I = bits 0–9, Q = bits 10–19, upper 12 bits changing
status bits) straight into SRAM; there is no DMA descriptor chain.

**Where 16380 comes from.** The write window is 64 KiB (16384 words): address
= `0x40820000 + 0x10000·owner bit 16 + 4·(position mod 16384)`. The stock path
uses the upper half with four canary words → 16380. The counter is 17 bits
wide; counts above 65535 run forever (never "done").

**Sample rates.** Dividers 0–5 = 80/40/20/10/8/4 MS/s. The lower rates simply
drop samples (no anti-alias filter): everything inside the analog passband
(`BANDWIDTH 0` ≈ 40–55 MHz) folds into the band at 4/8 MS/s. `BANDWIDTH 11`
attenuates signals 8/10/20 MHz away by only ≈13–19 / 18–28 / 29 dB. Dividers
6 and 7 produce 80 MS/s again: there is no hardware rate below 4 MS/s.

**The 1.7 ms per snapshot.** `adctrig()` adds a fixed ≈1.68 ms (80 MS/s: 1886 µs
instead of 205 µs). Every access to register `0x600a20b4` ("source-15 gate")
stalls the bus for ≈0.82 ms and `adctrig()` does a read-modify-write on every
call. The bit is clear after boot and stays clear, so the engine checks it once
per boot.

**Transport.** The C5 USB Serial/JTAG is USB 2.0 full speed only: ≈1.216 MB/s
bulk in theory, **930–940 kB/s** measured with pure sending (the limit is host
polling, not the firmware). UART0 at 2 Mbaud carries ≈200 kB/s.

**Memory.** 384 KiB SRAM in three 128 KiB banks; bank 1
(0x40820000–0x4083ffff) is reserved for the dump. No PSRAM on most boards, and
the dump can only write into its owner bank. Free heap after boot ≈86 KB
(largest block ≈51–59 KB depending on the build).

**Reading the dump bank while it is written.** While the modem owns bank 1,
CPU reads of *both* 64 KiB halves return wrong data (values from a different
memory, periodic in the address pattern; `IDLETEST`), and so do GDMA reads
(`GDMATEST`, diagnostics build). The ownership granularity is the whole
128 KiB bank. A true ping-pong ring on the dump bank is therefore impossible
(an early prototype that seemed to work streamed constant values). Instead,
a finished dump re-arms within **≈4 µs** (99.9 % duty back to back without
work, `B2B`), and copying a block out costs ≈2.6 CPU cycles per word. This is
the basis of the [continuous engine](#continuous-engine).

**LO range.** The receive LO does not track below ≈2.1 GHz: lower `FREQ`
values receive at a different LO around 2.2 GHz (a 1090 MHz setting listens
at ≈2186–2234 MHz). Exact tuning with the stock path: ≈2130–2730 and
4790–≈5745 MHz; with the Wi-Fi driver path up to at least 5990 MHz
([TUNEM](#tuning-tunem)). Board-to-board differences of a few dB and
occasional board-specific dead spots (e.g. 4960–5020 MHz on one board) were
seen.

## Commands

All new commands are listed in `CAPS` by the token in brackets. Rate index
= divider 0–5 (80/40/20/10/8/4 MS/s). Times are µs since boot (`TIME?`).

| Command | Reply | Notes |
|---|---|---|
| `CAP16/CAP20 <n> <rate>` (upstream) | `DATA n crc us` + payload | n ≤ 32704 [BIGCAP] |
| `CAPT16/CAPT20 <n> <rate>` [CAPT] | `DATAT n crc us t0_us seq lag` + payload | n 256–32704, t0 = time of sample 0, lag = flip lag dropped |
| `DDC8/DDC16 <n_out> <out_hz> [<offset_hz> [<src_idx>]]` [DDC] | `DATAT …` + int8/int16 LE I,Q | R = src/out integer 2–128; default source 20 MS/s (else 40/80); n limited to (32704−taps)/R; clean band ≈ ±38 % of the output rate |
| `SPEC <rate_idx> <nfft> <navg>` [SPECDB] | `SPEC nfft crc us t0` + nfft × int16 | 0.01 dB, −fs/2…+fs/2, nfft 64–1024, nfft·navg ≤ 16380 |
| `SWEEP <f0> <f1> <step> <rate_idx> <nfft> <navg>` [SWEEP] | per step `SWP mhz nfft crc us` + bins, then `END SWEEP steps us` | ≤ 2000 steps, any byte stops |
| `FREQK <kHz>` / `FREQK?` [FREQK] | `OK` / `FREQK <kHz>` | 1 kHz steps (integer MHz + LO offset); `FREQ` resets the offset |
| `TEMP?` [TEMP] | `TEMP 43.9` | on-chip sensor |
| `FWOPT?` [FWOPT] | `FWOPT 0.7.0 espsoup rx-only C5` | |
| `ENGINE STOCK/FAST`, `ENGINE?` | `OK` | STOCK = vendor `adctrig()` for ≤ 16380 samples (A/B comparison) |
| `STREAM8/STREAM16 <out_hz> [<offset_hz> [<n_max> [<dcblock> [<hw_idx>]]]]` [STREAM] | `OK STREAM rate bits hw R1 R2 taps words`, frames `BLK seq idx n crc t_us gap_in` + payload, `END STREAM frames samples lost_in overruns why` | 1 k–500 kS/s, default hardware rate 8 MS/s; `gap_in` = input samples missing before the frame |
| `CSPEC <rate_idx> <nfft> <navg> [<mode> [<stride>]]` [CSPEC] | `OK CSPEC fs nfft navg mode words lines`, per block `SPL seq t_us lines nfft mode gap crc` + lines × nfft u8, `END CSPEC seq blocks lost overruns why` | nfft 16–256, mode 0 mean / 1 max (default) / 2 both; u8 = 2·10·log10(\|X\|²) (0.5 dB) + 12·sh (sh = log2(nfft)−6 for nfft > 64) |
| `CPKT <rate_idx> <dec> <thr_x10> <pre> <len>` [CPKT] | `OK CPKT fin dec thr pre len words`, per burst `PKT seq t_us n dec crc flags peak` + n u8 magnitudes, `END CPKT …` | flags 1 = cut at the block end |
| `CGRID <t0_us> <period_us>`; during a run `CADJ <±us>` [CGRID] | `OK` | block starts on a time grid (two boards cover each other's gaps); `CGRID 0 0` free-running |
| `CSTAT?` [CONT] | `CSTAT blocks words lost maxgap duty slips late copy wait work fft lines` | last continuous run, cycles per block |
| `TUNEM <0,2,3,4,5>` / `TUNEM?` [TUNEM] | `OK` / `TUNEM n` | tuning path, default 4 (auto) |
| `VFM <rate_idx 2/3> [<D> [<px> [<std> [<pol> [<sel> [<kBps>]]]]]]` [DEMODFM] | see [VFM](#vfm--fm-video-line-stream) | |
| `GFSK <rate_idx> <R> <kbps> <mode> [<len> [<sync_hex> [<nbits> [<maxerr> [<chan> [<okonly>]]]]]]` [GFSKPKT] | see [GFSK](#gfsk--packet-catcher) | |
| `DSTAT?` | statistics of the last VFM/GFSK run | |
| `DCCAL`, `DCCAL?`, `DCAUTO 0/1` [DCCAL] | `DCCAL before i q after i q base i q best i q steps n jac a b c d auto x` | [DC calibration](#hardware-dc-calibration) |
| `PCAP`, `PLSB`, `PLEVEL?`, `PRUN`, `PSTAT?`, `PDC`, `VFMSRC`, `STRSRC`, … [PARLIO] | see [PARLIO](#commands-parlio) | |

Diagnostics (not in `CAPS`): `TIME?`, `HEAP?`, `HEAPMAP?`, `LASTCAP?`,
`ARMPROF?`, `IDLETEST <div>`, `DSPBENCH`, `STREAMSTAT?`,
`CSBENCH <nfft> <navg> <mode>`, `CSDATA <rate> <nfft> <navg> <mode>`,
`DMBENCH`, `VFMDATA`, `GFDATA`, `DMLOAD`, `DMVFM`, `DMGF`, `DMPOL`, `PBENCH`,
`PDIAG`, `PMAP`, `PPINS`, `PDRIVE`, `PEDGE`, `PMODEM`, `PCFG?`. A build with
`-DC5_GAP_PROBE=ON` adds `GDMATEST`, `RATETEST` and `B2B` (never packaged).

## Measurements

Native USB, one board per measurement series, release builds.

**Snapshots with the upstream commands (CAP16/CAP20, 16380 samples, 12 repeats):**

| Rate | Stock 8 bit: cycle / fps / kB/s | ESPsoup 8 bit | Stock 10 bit fps | ESPsoup 10 bit fps | Capture time stock → ESPsoup |
|---|---|---|---|---|---|
| 80 MS/s | 42.0 ms / 23.8 / 780 | 38.6 ms / 25.9 / 848 | 18.9 | 20.4 | 1886 → 206 µs |
| 40 | 42.0 / 23.8 | 38.6 / 25.9 | 18.7 | 20.4 | 2089 → 411 µs |
| 20 | 42.3 / 23.6 | 39.0 / 25.6 | 18.5 | 20.0 | 2499 → 821 µs |
| 10 | 43.0 / 23.3 | 39.9 / 25.1 | 18.3 | 19.6 | 3318 → 1640 µs |
| 8 | 43.8 / 22.8 | 40.0 / 25.0 | 18.1 | 19.6 | 3727 → 2049 µs |
| 4 | 45.9 / 21.8 | 42.1 / 23.8 | 17.5 | 18.9 | 5778 → 4097 µs |

Live view in a client over the same link: 1024 samples 87 → 103 fps, 4096:
39 → 45 fps, 16380: 18.2 → 20.0 fps.

**Long snapshots (BIGCAP, 32704 samples):** 80/40/20/10/8/4 MS/s =
0.41/0.82/1.64/3.27/4.09/**8.18 ms** contiguous (stock: at most 4.10 ms at
4 MS/s). The dump fills the lower half, flips the owner bit at position 16384
and continues in the upper half; the flip lag (1–45 samples) is measured
exactly from sentinels and dropped, then the data is reordered in place.
8 bit: 13.2–11.9 snapshots/s, 10 bit: 10.3–9.5/s. 64k–128k snapshots are
impossible (2 × 64 KiB window, no PSRAM).

**DDC (from 20 MS/s):** 4 MS/s → 6524 samples (1.63 ms, 64 ms per snapshot),
2 MS/s → 3254, 1 MS/s → 1619, 500 kS/s → 801, 250 kS/s → 395. Stage: DC
removal, NCO, Kaiser FIR with 16·R+1 taps (≤ 1025), decimation by R.

**Anti-alias test at 2120 MHz** (strong LTE carriers at 2130–2150 MHz next to
the band, gain 58, mean noise/alias density in the band): raw 4 MS/s `BANDWIDTH 0`
−90.9 dBFS/Hz · raw 4 MS/s `BANDWIDTH 11` −97.2 · raw 8 MS/s −95.0 · DDC 4 MS/s
from 20 MS/s −95.8 · **from 40 MS/s −99.7 · from 80 MS/s −99.6**. At 2135 MHz:
raw 4 MS/s −89.1 → DDC −102.4. The folded wideband/LTE energy disappears
(8.8–13 dB lower, flat floor). From 20 MS/s the +20 MHz neighbour folds onto
0 Hz, so use source 40/80 MS/s (`src_idx` 1/0) for clean results.

**SPEC/SWEEP:** `SPEC 0 256 16` 7.0 ms per call; `SWEEP 100 6000 40 0 256 8` =
148 steps in **0.69 s** (a panorama from single snapshots: 7.0–7.4 s, ≈10×
slower).

**FREQK** (shift of a 2140 MHz LTE spectrum, 20 MS/s, 9.8 kHz bins): +100 →
−88 kHz, −100 → +137, +250 → −254, −300 → +293, +500 → −498 (±30 kHz
measurement uncertainty).

**TEMP?:** 42–49 °C while running.

## Continuous engine

`cont_t` with `cont_start/cont_next/cont_stop` (`c5_cont.h`): block k+1 is
dumped while the CPU processes the heap copy of block k (8192 words).
`cont_next` waits for "done", returns bank 1 to the CPU, copies the block and
immediately starts the next dump. The only gap per block is the copy and the
restart (plus processing time if it exceeds one block). Gaps are counted
**exactly with the CPU cycle counter** (CPU and dump clock share the crystal:
60/30/24/12/6/3 cycles per sample at 4/8/10/20/40/80 MS/s) and reported as
missing input samples. Memory: the block copy (32 KiB) and the USB ring
(16 KiB) must fit into the largest heap block (else 4096-word blocks). Output
goes through a byte ring drained by the USB Serial/JTAG IN-empty interrupt.

**STREAM** (duty = share of input samples processed, 16 bit; 8 bit within ±1 %
except where USB limits):

| Output rate | hw 4 MS/s | 8 MS/s | 10 MS/s | 20 MS/s |
|---|---|---|---|---|
| 25 kS/s | 95.7 | 91.8 | 89.9 | 64.0 |
| 50 kS/s | 95.7 | 91.8 | 89.9 | 63.5 |
| 100 kS/s | 95.7 | 91.3 | 87.8 | 64.1 |
| 200 kS/s | 94.8 | 90.4 | 87.6 | 55.5 |
| 250 kS/s | 93.9 | 88.9 | 79.5 | 52.1 |
| 500 kS/s | 47.0 (USB, 937 kB/s) | 47.0 (USB) | 46.7 (USB) | 39.7 |

8 bit at 500 kS/s: 81.3 / 67.9 / 61.7 / 44.0 % (813/678/617/440 kB/s). At
20 MS/s the CPU is too slow (CIC ≈15 cycles/sample > 12). For comparison, the
earlier snapshot-based stream reached 54–75 %.

**STREAM continuity** (own spur at 2400.000 MHz, `FREQK 2400030`): `idx` and
`seq` gap-free, `t_us` monotonic; Σ `gap_in` − start-up = `CSTAT lost` =
`END … lost_in` exactly; `t_us` follows the sample time axis counted from
`gap_in` within −0.1…+7 µs over 5 s (≈1.4 ppm). `offset_hz` is the host
frequency moved to 0 Hz; the NCO stays phase-continuous across gaps. The
CIC/FIR state is carried across a gap, so the first ≈ taps/R2 outputs after a
gap mix samples from before and after it (coherent users drop them).

**CSPEC** (duty %, mode max, stride 1 / 2 / 4; navg chosen so the output stays
≲ 0.45 MB/s):

| nfft | 4 MS/s | 8 MS/s | 10 MS/s | 20 MS/s |
|---|---|---|---|---|
| 16 | 95.0 / 95.7 / 95.7 | 61.2 / 90.9 / 91.8 | 49.0 / 84.1 / 89.9 | 25.6 / 45.3 / 73.6 |
| 32 | 94.8 / 95.7 / 95.7 | 59.9 / 90.3 / 91.8 | 47.9 / 82.9 / 89.9 | 24.9 / 44.3 / 72.8 |
| 64 | 94.8 / 95.7 / 95.7 | 57.1 / 90.3 / 91.8 | 45.7 / 79.7 / 89.9 | 23.7 / 42.4 / 70.4 |
| 128 | 88.3 / 95.7 / 95.7 | 46.7 / 83.5 / 91.8 | 37.3 / 66.8 / 88.7 | 18.6 / 33.3 / 55.1 |
| 256 | 83.5 / 95.7 / 95.7 | 41.7 / 72.5 / 90.2 | 33.4 / 58.0 / 87.7 | 16.7 / 29.0 / 46.1 |

Mode 2 (mean + max), nfft 64: 4 MS/s 86.5 / 94.8 / 95.7 %, 8 MS/s
48.7 / 81.8 / 90.3 %, 10 MS/s 39.0 / 65.4 / 88.2 %. The CSPEC FFT
(`cspec_dsp.h`) is a mixed radix-8/4 decimation-in-frequency FFT with the
unpacking and Hann window fused into the first stage and |X|² accumulation in
the last: **46 cycles/sample at nfft 64** instead of ≈115 with radix 2.
Verification: host test against a double-precision DFT (mean error 0.3 u8);
`CSDATA` on real data: device = host FFT of the same samples in 85–93 % of
the bins exactly, ±1 u8 in ≥ 99.5 %; continuous CSPEC against snapshots:
noise floor within 0–0.8 dB, overall RMS difference 0.8 dB.

**CPKT** (FREQ 2437, threshold 4×, pre 16, len 256): 4 MS/s dec 4 95.7 % ·
8 MS/s dec 8 91.6 % · 10 MS/s dec 8 89.3 % · 20 MS/s dec 16 55.1 %. Bursts:
BLE advertising at 2426 MHz ≈370/s, Wi-Fi channel 6 ≈310/s; timestamps
monotonic, CRC ok.

**Two boards.** With `CGRID` on both boards and `CADJ` steering, two C5 boards
tuned to the same frequency covered **100.000 % of 180 s** at 4 MS/s
(envelope correlation on Wi-Fi bursts plus crystal feed-forward; host tool
not part of this repository).

## Tuning (TUNEM)

`TUNEM <m>` selects how the PLL is programmed; `TUNEM?` reports it.

| m | Path |
|---|---|
| 0 | upstream esp-sdr path: band-calibration channel + `phy_set_rf_freq_offset` (kHz offsets via `FREQK`) |
| 2 | Wi-Fi driver path: `esp_wifi_set_channel` on a nearby channel, then libphy `phy_change_channel(MHz)` (the approach of esp_wifi-based 802.11p/ITS-G5 receivers) |
| 3 | as 2 with the baseband in 802.11p mode (`phy_11p_set`), diagnostic only |
| 4 | **auto (default):** 2 above 3000 MHz, 0 at 2.4 GHz and whenever a `FREQK` kHz offset is set |
| 5 | experiment: nearest public Wi-Fi channel + `phy_set_freq(MHz, 0)` (sequence as in the C5VRX project); measured identical to 4 |

Measured with a CW generator (tone on/off, confirmed by a 0.5 MHz LO shift),
10 MHz steps, "confirmed at the median gain", generator level −50/−30/−10 dB:

| Path | 2.4 GHz | LTE B7 | UNII-1/2 | UNII-2e | UNII-3 | ITS-G5 | 5.93–5.99 GHz |
|---|---|---|---|---|---|---|---|
| 0 esp-sdr | 9/9 @−30 | 8/8 @−30 | 21/21 @−30 | 15/26 @−10 | 8/13 @−10 | 1/8 @−10 | 0/7 |
| **2 Wi-Fi driver** | 9/9 @−30 | 8/8 @−30 | 21/21 @−30 | **26/26 @−30** | **13/13 @−30** | **7/8 @−30** | **7/7 @−30** |
| 3 802.11p | 3/9 @−10 | 3/8 @−10 | 11/21 @−10 | 9/26 @−10 | 3/13 @−30 | 0/8 | 3/7 @−10 |

The passband (tone at ±5…±35 MHz, 80 MS/s) at 5500 MHz is the same for 0 and 2
(flat ±20 MHz, −20 dB from ±25 MHz); at 5900 MHz path 0 is practically deaf,
path 2 has 11–18 dB above ±25 MHz. 5990 MHz is the limit of the generator, not
necessarily of the chip. On the Wi-Fi driver path the PHY channel mode chosen
by `BANDWIDTH` is not applied (the analog filter setting is). A PLL bias
experiment that locked the LO up to 6.4 GHz (RF I2C 106/0[7:4]) left the
receiver deaf above 5.6 GHz and was removed. No path tunes below ≈2.1 GHz.

## On-device demodulators

Why on the device: analog FPV video needs ≥ 10–20 MS/s × 2 B (20–40 MB/s), GFSK
at 1–2 Mbit ≥ 8 MB/s; USB full speed carries ≈0.94 MB/s. The demodulators
send lines or packets instead of samples. Narrow-band FM/FSK (≤ 250 kS/s) does
not need them (`STREAM` at 89–95 % duty). The DSP (`demod_dsp.h`) is
integer-only and compiles unchanged on the host (bit-exact test vectors).

### VFM – FM-video line stream

`VFM <rate_idx> [<D> [<px> [<std> [<pol> [<sel> [<kBps>]]]]]]`

* `rate_idx` 2 = 20 MS/s (default `D` = 2) or 3 = 10 MS/s (`D` = 1). With
  `VFMSRC 1` (default) the samples come from the [PARLIO path](#gap-free-parlio-path).
* `D`: phase of every D-th sample, wrapped phase differences over D samples
  summed per 0.2 µs bin (telescoping, so the noise of a bin depends only on its
  end points). Valid for |f − LO| < fs/(2D). Phase via a 64×64 arctangent table
  of the 6-bit field [qs, qs+5] of I and Q, qs per block from the amplitude.
* `px` 16–256 pixels per line (default 128) over the active picture (PAL
  10.5–62.5 µs, NTSC 9.4–62.0 µs after the sync edge); box average of the
  instantaneous frequency.
* `std` 0 auto (64.0 / 63.56 µs line period), 1 PAL, 2 NTSC; `pol` (host
  orientation) 0 auto, 1 = sync tip at the lowest frequency, 2 = highest.
* `sel` bit 0 (default 1) = do not send vertical-blanking lines, bit 1 = whole
  line (sync to sync). `kBps` USB budget (default 850, 0 = unlimited).

Per block: discriminator → 0.2 µs bins → acquisition (histogram quantiles,
3.4–6 µs pulses at the line period) → flywheel: sync search ±0.6 µs around the
expected edge, line reference = prediction + ¼ of the error, line period
tracked (±0.5 %), sync-tip and back-porch levels averaged; vertical sync from
broad-pulse patterns gives ITU line numbers (PAL 1–625, NTSC 1–525).

```
OK VFM <fin> <D> <px> <std> <pol_dev> <sel> <rate_Bps> <words>
VFL <seq> <t_us> <nlines> <px> <gap_in> <crc32> <tip_khz> <blank_khz> <st>   + nlines × (8 + px) bytes
END VFM <frames> <blocks> <lost_in> <overruns> <lines> <lines_sent> <lines_dropped_rate> <why>
```

Line record (little endian): `u16 line` (0 = no vertical sync yet), `u8 flags`
(1 SYNC measured, 2 FIELD2, 4 VBI, 8 VLOCK, 16 VSYNC, 32 NTSC, 64 FULL),
`u8 field_count`, `u32 t16` (sync edge in 1/16 µs since boot, mod 2³²), then
`px` bytes: composite level 0 = sync tip, 64 = blanking, 213 = white.
`st` bit 0 locked, 1 vertical sync locked, 2 NTSC, 3 sync tip at the highest
frequency. Practical notes: tune the LO about 2 MHz below the video carrier
(a carrier exactly at DC is suppressed), use a fixed gain (`GAIN MANUAL`).

### GFSK – packet catcher

`GFSK <rate_idx> <R> <kbps> <mode> [<len> [<sync_hex> [<nbits> [<maxerr> [<chan> [<okonly>]]]]]]`

* `rate_idx` 3/4/5 (10/8/4 MS/s), 2 (20 MS/s) only with R = 4. Box filter and
  decimation by `R`; samples per bit = fin/R/bitrate must be 2, 4, 8 or 16.
  BLE 1 Mbit: `GFSK 4 2 1000 0` (8 → 4 MS/s, 4 sps) or `GFSK 5 1 1000 0`.
* `mode` 0 = BLE (access address, default 8E89BED6, de-whitening with the
  channel of `FREQ` (2402/2426/2480 → 37/38/39, data channels 0–36) or `chan`,
  length from the header, CRC-24 checked), 1 = sync word (`sync_hex`,
  `nbits` 8–32, MSB first, then `len` bytes), 2 = preamble (≥ `nbits`
  alternating bits, then `len` bytes; the host searches the address).
* `maxerr` 0–2 bit errors in the sync word (default 1), `okonly` 1 = only
  CRC-valid BLE packets.

```
OK GFSK <fin> <R> <fout> <sps> <mode> <sync_hex> <nbits> <maxerr> <len> <chan> <words>
GPK <seq> <t_us> <n> <flags> <rssi_dbfs×10> <cfo_hz> <dev_hz> <errs> <crc32>   + n bytes
END GFSK <packets> <blocks> <lost_in> <overruns> <hits> <decoded> <crc_ok> <why>
```

flags 1 CRC ok (BLE), 2 cut at the block end, 4 BLE PDU, 8 BLE length > `len`.

**Host tests** (`tests/espsoup/`): VFM PAL 20 MS/s luminance error 1.05 steps
(of 149) against the picture model, edge error 30 ns, line numbers exact; NTSC
inverted, 12 dB SNR, +60 ppm transmitter crystal and noise-only cases pass.
GFSK: BLE 8 MS/s R=2 217/217 complete packets, 4 MS/s 247/252, 8 MS/s R=1
211/213; BLE sensitivity (8 MHz noise bandwidth) 11 dB 100 %, 7.9 dB 95 %,
4.9 dB 44 %; 2 Mbit GFSK with sync word 98–99 %; 250 kbit 2-FSK 100 %.

**Hardware.** Device and host results are bit-identical on all test vectors
(`DMLOAD/DMVFM/DMGF`) and on real snapshots (`VFMDATA/GFDATA`). Cycles per
input sample (`DMBENCH`, table in SRAM): discriminator D=2 @20 MS/s 7.2–7.6
(budget 12), D=1 @10 MS/s 15.0–15.4 (24); GFSK scan R=2/4 sps 20.1–21.7
(8 MS/s, budget 30). VFM locked on the dump engine: 12.9 cycles/sample at
20 MS/s, 24.3 at 10 MS/s. The arctangent table must be in SRAM (from flash
+1.5–2.5 cycles/sample) and the bins must not be in LP SRAM (+2–3).

GFSK live (BLE advertising, 10 s per channel, `GFSK 4 2 1000 0`, `GAIN MANUAL 60`):

| Channel | packets/s | CRC-ok/s | MACs in 10 s | duty | USB |
|---|---|---|---|---|---|
| 2402 (37) | 106 | 62 | 28 | 90.3 % | 9.1 kB/s |
| 2426 (38) | 109 | 72 | 30 | 90.2 % | 9.4 kB/s |
| 2480 (39) | 127 | 78 | 30 | 89.9 % | 10.9 kB/s |

**The hardware AGC breaks GFSK:** with `GAIN HARDWARE` only 14 CRC-ok/s at
2426 MHz, with `GAIN MANUAL 60` 76/s (70: 49, 50: 59, 40: 57, 30: 47) — the AGC
changes the gain inside packets. Use a fixed gain for GFSK and VFM.

## Hardware DC calibration

`dc_cal.h` ports `receiver_dc_calibrate()` of the S31 streaming firmware
(`main/targets/esp32s31/streaming/dc.c`) to the C5: the C5 libphy
`phy_pbus_set_dco` writes the same PBUS fields (block 2/3, bank 1/2). The DC
is measured from a 20 MS/s snapshot (16380 samples, ADC8 counts), the PBUS is
put into debug mode with the live values, the 2×2 response of the two DC DACs
(±16 codes) is measured, up to five Newton steps with step halving follow and
the best measured codes are latched. Measured at 6 frequencies × gain 40/60/70:
2.2–2.7 GHz 1.4–3.5 → 0.2–0.6 counts (9–18 dB); **5.2–5.8 GHz at gain 40/60:
4–43 → 0.07–0.44 counts (27–47 dB)**; at gain 70 at 5 GHz (ADC at its limit)
only 128 → 58–60. Duration 50–100 ms.

Limits: the correction lasts until the next `prepare_rx()` (`FREQ`,
`BANDWIDTH`, `LPF`, `TUNEM` return the PBUS to work mode) or the next `GAIN`
command, and it needs a manual gain (in debug mode the PBUS does not follow the hardware AGC).
`DCAUTO 1` calibrates before every `VFM`/`STREAM`/`GFSK`/`CSPEC` run (default
off, so the dump-engine outputs are unchanged); PARLIO runs calibrate by
default (`PDC 1`). The upstream `DC?`/`DC 0|1` (spectrum DC tracker) is a
different, software feature and works as before.

## Gap-free PARLIO path

The modem diagnostic bus carries the dump word: `MODEM_DIAG[0:9]` = I,
`MODEM_DIAG[10:19]` = Q (10-bit two's complement, device orientation; bits
8/9 and 18/19 are equal for small signals; a +4.075 MHz tone shows
Phase(Q) − Phase(I) = −91°; spectra match CAPT20). Routed to pads through the
GPIO matrix with output **and** input enabled, PARLIO RX samples them
internally — no wires. The C5 PARLIO RX has only 8 data lines (the S31 has 16)
and delivers at most 40 MB/s (80/60 MHz requested → 40.000 MB/s measured), so
the path carries **40 MS/s with 4 bits of I and 4 bits of Q**:
`byte = I[lsb+3:lsb] | Q[lsb+3:lsb] << 4`.

* Pad consistency (the same diagnostic signal on all 8 lines): GPIO 0–10, 23,
  24 = 100 %, GPIO 25 98.2 %, GPIO 26 94.8 % (not used). Default pins
  `0 1 3 4 5 7 10 23` (USB 13/14, flash/PSRAM 15–22, UART 11/12, LED 27,
  BOOT 28 stay free; header pads on the ESP32-C5-DevKitC-1 and the Seeed
  XIAO ESP32-C5).
* **Caution:** during a PARLIO run these 8 pads are driven as outputs and
  toggle at up to 40 MHz (lowest drive strength). Do not use the PARLIO path
  (`VFMSRC 0`, `STRSRC 0`, no `PCAP`/`PRUN`/`PDIAG`) on a board with anything
  connected to those pins, or move them with `PPINS`.
* Window: the top 4 bits [9:6] are too coarse for normal levels. CW tone SNR at
  gain 60: [9:6] 5 dB, [6:3] 17.9 dB, 10-bit dump 18.9 dB; gain 70: [7:4]
  18.6 dB vs 19.0 dB. A window too low wraps. `PLSB AUTO` (default) takes two
  16 KB snapshots of I[9:2]/Q[9:2] before each run and picks the lowest window
  without wrapping (99.9 % point; median-based for VFM, because a Wi-Fi burst
  in the snapshot otherwise chose lsb 6 for a weak carrier).
* Gap-free: an infinite PARLIO transaction (soft delimiter, EOF = ring length)
  into a 57344-byte ring in SRAM bank 1 (free while the CPU owns it, no heap).
  `PRUN 62`: 2.48·10⁹ bytes, 0 overruns, bytes vs. `esp_timer` +0.006 ppm
  (a stall of one clock per ring wrap would show as −17 ppm); the rest of
  bank 1 stays untouched. During a run the Wi-Fi promiscuous filter is set to
  0 (no frames to the driver task) and restored afterwards.

### Commands (PARLIO)

| Command | Reply / effect |
|---|---|
| `PCAP <n> [<div> [<where>]]` | snapshot of n bytes (≤ 65532, 40 MS/s): `PDATA n crc t0_us lsb nodes` + bytes |
| `PLSB <0..6>` / `PLSB AUTO` | fixed / automatic 4-bit window |
| `PLEVEL?` | `PLEVEL lsb peak_lsb` |
| `PRUN <s> [<ring>]` | endless ring, the CPU reads every word; once per second `PST t_us bytes nodes eofs overruns ppm xor`, then `END PRUN …` |
| `PSTAT?` | last PARLIO run: `blocks bytes lost overruns late duty maxlag lsb peak sec rate cpb` |
| `PDC 0/1` | DC calibration before every PARLIO run (default 1, manual gain only) |
| `VFMSRC 0/1`, `VFMSRC?` | source of `VFM`: 1 = PARLIO (default), 0 = dump engine |
| `STRSRC 0/1`, `STRSRC?` | source of `STREAM8/16` without an explicit hardware rate: 0 = dump engine (default), 1 = PARLIO |
| `PMAP`, `PPINS`, `PDRIVE`, `PEDGE`, `PMODEM`, `PCFG?`, `PDIAG`, `PBENCH` | diagnostics: line map, pads, drive strength, sampling edge, dump mode, bus activity, cycles |

**VFM on PARLIO** (`vf_disc_p8`, `vf_block_p8`, protocol unchanged): the phase
of a byte is one table lookup; phase points at bytes 3 and 7 of each 0.2 µs
bin give the same bin scale as 20 MS/s D=2. The last 340 bins stay in front of
each new window, so no line is lost at block boundaries. `OK VFM` reports
`fin` 20000000 and `words` 8192 (bytes per block); one `VFL` frame per four
blocks. CPU: ≈5.2–5.8 cycles per byte of a budget of 6.

Live test (PAL test picture, FM-modulated by a signal generator at 2434 MHz,
LO 2436 MHz, gain 50):

| Run | lines processed/s | sent/s | duty (input) | noise (steps) | one frame filled |
|---|---|---|---|---|---|
| PARLIO, px 48, 800 kB/s, **60 s** | **15 625** | 14 213 (all active lines) | **100.00 %, 0 overruns** | – | – |
| PARLIO, px 48, 800 kB/s, 15 s | 15 586 | 14 182 | 100.00 % | 6.35 | 99.8 % |
| PARLIO, px 40, all lines incl. VBI | 15 563 | **15 563** | 100.00 % | 6.03 | 100 % |
| PARLIO, px 128, 850 kB/s | 15 602 | 6 243 (USB) | 99.99 % | 7.93 | 72.9 % |
| dump engine, px 48, 800 kB/s | – | 8 907 | 71.2 % | 9.86 | 99.8 % |
| dump engine, px 128 | – | 6 243 (USB) | 59.9 % | 7.07 | 76.9 % |

60 s in detail: 937 513 lines (= 15 625/s, the PAL nominal rate), 852 813 sent,
852 810 received, 0 CRC errors, 0 sequence gaps, VLOCK 99.98 %, 0 relocks,
`lost 0 overruns 0 late 0`, 5.41 cycles/byte. With px 128 USB limits the output
to ≈6250 lines/s; the full 15 625 lines/s reach the host only with px ≤ 48.

**STREAM on PARLIO** (`STRSRC 1`): boxcar-16 (SWAR) → 2.5 MS/s → the same
CIC3/NCO/FIR path; output rates 2.5 MHz/(R1·R2) with R2 ∈ {2, 4, 8, 5} (e.g.
250k, 125k, 100k, 50k, 25k, 20k, 10k; not 200k/500k). **25 and 50 kS/s: 100.0 %
duty, no gaps** (dump engine 91–96 %); 100 kS/s 75 %, 250 kS/s 77 % (CPU-bound),
hence `STRSRC 0` stays the default. Level: 10-bit LSB scale for window lsb 6,
2^(6−lsb) higher below.

**Spectrum from PARLIO data** (host FFT of `PCAP` bytes against CAPT20/SPEC):
the same signals at the same place from 2100 to 5990 MHz; with a suitable
window the tone SNR is within 1 dB of the 10-bit dump; outside ±10 MHz the
4-bit floor is flat (quantisation noise).

**Memory.** With the GDMA/PARLIO driver code in IRAM (≈4.4 KB) the largest
heap block dropped to 47 KB and STREAM/CSPEC fell back to 4096-word blocks;
the C5 defaults keep that code in flash (only the discriminator is in IRAM).

## Validation of version 0.7.0

Re-measured after rebasing onto the current upstream (one ESP32-C5 DevKitC-1
type board, 4 MB flash, native USB):

* Memory: free heap 90.6 KB, largest block 69.6 KB after boot (internal 0.6:
  76.6 / 51.2 KB) — the shared spectrum workspace lives in the dump bank and the
  C5 uses 4 static Wi-Fi RX buffers. After many continuous runs the largest
  block was still ≥ 48 KB (8192-word blocks).
* Upstream spectrum protocol: `tools/check_spectrum.py --stats` passes all 48
  rate/size/detector combinations; snapshot FFT frames/s at 80 MS/s with
  256/512/1024/2048 bins: 377/201/106/54 (with `ENGINE STOCK`: 228/148/87/48).
* Captures: CAP20 16380 at 20 MS/s 821 µs (stock 2504 µs); BIGCAP 32704 samples
  411/821/1640/3278/4098/8193 µs at 80…4 MS/s; `SWEEP 100 6000 40` 0.74 s.
* Continuous engine: STREAM16 200 kS/s 90.5 %, 50 kS/s at 4 MS/s 95.7 %, CSPEC
  4 MS/s nfft 64 94.7 %, 8 MS/s stride 2 90.4 %, CPKT 8 MS/s 91.6 %; GFSK BLE
  at 2426 MHz 90.5 % duty (112 packets/s, 39 CRC-valid/s in this environment).
* Demodulators: all device test-vector blocks (PAL 20/10 MS/s, NTSC inverted,
  BLE 8/4 MS/s, 2 Mbit GFSK, 250 kbit 2-FSK; 40 blocks each) and real snapshots
  are bit-identical to the host build.
* PARLIO: `PRUN 10` 4.0·10⁸ bytes, 0 overruns, bank 1 untouched (+1.6 ppm byte
  count vs. timer with the ISR in flash). VFM with the PAL test picture
  (2434 MHz carrier, LO 2436 MHz): PARLIO 15 515–15 558 lines/s processed,
  14 181 sent/s at px 48 and 15 515 sent/s at px 40 incl. VBI, 100 % input duty;
  the dump engine sent 8 870 lines/s (70 % duty).
* TUNEM on this board (CW tone at −10 dB generator level, tone over median
  noise): 5500 MHz 19.7 / 19.9 dB (path 0 / auto), 5745 19.6 / 22.6, 5800
  15.5 / 19.6, 5900 8.7 / 10.2, 5990 11.9 / 14.3 dB. The advantage of the Wi-Fi
  driver path is board dependent (1.5–4 dB here, ~20 dB on another board).
* DC calibration at 5500 MHz, gain 60: 12.0 → 0.35 ADC8 counts. A `GAIN`
  change after a calibration now returns the PBUS to work mode (before, the
  forced PBUS values kept the old gain active).
* The ESPsoup app (snapshot spectrum, device sweep, LTE calibration, CSPEC live
  view, burst catcher, live FPV video, BLE/Remote ID/nRF24 via the packet
  catcher) works unchanged with this version.

## Limitations

* **Truly gap-free 10-bit streaming** is impossible on the dump path (see
  Hardware findings); the continuous engine reports every gap exactly. The
  PARLIO path is gap-free but 4+4 bit and CPU-bound above ≈50 kS/s output.
* **USB full speed** limits raw output to ≈0.94 MB/s; more bandwidth would need
  external hardware.
* **No narrow hardware filter** before the dump (the LPF modes are ≈20 MHz
  filters); no hardware rate below 4 MS/s.
* **Tuning** below ≈2.1 GHz is impossible (see LO range); ADS-B at 1090 MHz,
  GNSS L1 and similar bands cannot be received.
* **Hardware AGC** changes the gain inside packets: use `GAIN MANUAL` for
  GFSK/VFM and for the DC calibration.
* Not implemented: video sound sub-carrier, NBFM audio on the device (`STREAM`
  is sufficient), ELRS FLRC (fractional samples per bit), GFSK on the PARLIO
  path.
* Tested on five ESP32-C5 boards (DevKitC-1 type and Seeed XIAO ESP32-C5);
  UART-only operation of the continuous commands was not characterised
  (expected ≈100 kS/s STREAM output at 2 Mbaud).

## Testing

* Host: `python -m unittest discover -s tests` (needs a C compiler as `cc`;
  `node` for the VFM/GFSK scripts). `tests/test_espsoup_c5.py` checks the CSPEC
  FFT against a double-precision DFT, runs the VFM/GFSK demodulators on
  synthetic signals (`tests/espsoup/`, bit-exact with the device) and verifies
  the generated tables; the upstream command/tuning tests cover the C5 command
  parser including `TUNEM` and BIGCAP limits.
* Device: `DMLOAD`/`DMVFM`/`DMGF` replay the host test vectors on the device;
  `VFMDATA`/`GFDATA`/`CSDATA` return a snapshot together with the device
  result for host comparison; `CSTAT?`, `DSTAT?`, `PSTAT?` report duty and
  cycles of the last run.

## Sources and attribution

* Base: [ESPARGOS ESP-SDR](https://github.com/ESPARGOS/esp-sdr) (GPL-3.0-or-later).
* `dc_cal.h` is a port of `receiver_dc_calibrate()` from this repository's
  `main/targets/esp32s31/streaming/dc.c` (ESPARGOS, GPL-3.0-or-later).
* The PARLIO path (routing MODEM_DIAG to pads and reading them with PARLIO RX,
  muting the promiscuous filter during runs) and the `TUNEM 5` call sequence
  follow ideas of the **C5VRX** project by konradit (GPL-3.0-only). The code
  here is an independent implementation; no C5VRX code was copied. See
  [NOTICE](../NOTICE).
* The Wi-Fi driver tuning path uses the libphy call sequence known from
  esp_wifi-based 802.11p/ITS-G5 receivers.
