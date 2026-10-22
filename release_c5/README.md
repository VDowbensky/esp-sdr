# ESPsoup firmware 0.7.0 for the ESP32-C5

Receive-only ESP-SDR firmware for the ESP32-C5 with the ESPsoup extensions
(faster captures, BIGCAP, DDC, continuous capture, CSPEC/CPKT, tuning to
5990 MHz, on-device FM-video and GFSK demodulators, gap-free PARLIO path).
All ESP-SDR commands are unchanged; see docs/espsoup-c5.md in the source.

* Chip: ESP32-C5, flash 2MB or larger. Native USB (Serial/JTAG)
  recommended; UART0 (TX GPIO11 / RX GPIO12, 2 Mbaud) works as well.
* Identify: `INFO` -> `C5SDR 6 burst 16380`, `FWOPT?` -> `FWOPT 0.7.0 espsoup rx-only C5`.
* CAUTION: the PARLIO path (default source of `VFM`) drives GPIO 0 1 3 4 5 7 10 23
  as outputs while it runs. Do not use it on boards with anything connected to
  those pins (`VFMSRC 0` selects the dump engine).

## Flash

Back up the current flash first (adjust port and size):

    python -m esptool --chip esp32c5 -p PORT -b 921600 read-flash 0 ALL backup.bin

Then write the release (from this directory):

    python -m esptool --chip esp32c5 -p PORT -b 921600 write-flash @flash_args

or with explicit offsets:

    python -m esptool --chip esp32c5 -p PORT -b 921600 write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB 0x2000 bootloader.bin 0x8000 partition-table.bin 0x10000 espsoup-firmware-0.7.0-esp32c5.bin

Verify the files first: `sha256sum -c SHA256SUMS`.

## License and source

GPL-3.0 (ESP-SDR by ESPARGOS is GPL-3.0-or-later; see NOTICE in the source
for the ESPsoup additions and the C5VRX note). These binaries may only be
redistributed together with the corresponding source code: the source
commit is `0a7a405b370b349c056184950aa54db4e6e92c04`, built with ESP-IDF `see firmware-targets.json`.

Build date 2026-10-20.
