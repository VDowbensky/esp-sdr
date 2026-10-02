#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Package an ESPsoup ESP32-C5 build as a flat release directory.

Layout (the format the ESPsoup app's firmware bundler expects):
  bootloader.bin, partition-table.bin, espsoup-firmware-<version>-esp32c5.bin,
  manifest.json (schema 1, one variant), flash_args, SHA256SUMS, README.md.

The images and checksums come from tools/export_web_firmware.py, which also
refuses diagnostic probe builds.

  python tools/espsoup_release.py --build-dir build-c5 --version 0.7.0 --output out/
"""
import argparse
import datetime
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

from export_web_firmware import export

ROOT = Path(__file__).resolve().parents[1]

README = """# ESPsoup firmware {version} for the ESP32-C5

Receive-only ESP-SDR firmware for the ESP32-C5 with the ESPsoup extensions
(faster captures, BIGCAP, DDC, continuous capture, CSPEC/CPKT, tuning to
5990 MHz, on-device FM-video and GFSK demodulators, gap-free PARLIO path).
All ESP-SDR commands are unchanged; see docs/espsoup-c5.md in the source.

* Chip: ESP32-C5, flash {flash_size} or larger. Native USB (Serial/JTAG)
  recommended; UART0 (TX GPIO11 / RX GPIO12, 2 Mbaud) works as well.
* Identify: `INFO` -> `C5SDR 6 burst 16380`, `FWOPT?` -> `FWOPT {version} espsoup rx-only C5`.
* CAUTION: the PARLIO path (default source of `VFM`) drives GPIO 0 1 3 4 5 7 10 23
  as outputs while it runs. Do not use it on boards with anything connected to
  those pins (`VFMSRC 0` selects the dump engine).

## Flash

Back up the current flash first (adjust port and size):

    python -m esptool --chip esp32c5 -p PORT -b 921600 read-flash 0 ALL backup.bin

Then write the release (from this directory):

    python -m esptool --chip esp32c5 -p PORT -b 921600 write-flash @flash_args

or with explicit offsets:

{explicit}

Verify the files first: `sha256sum -c SHA256SUMS`.

## License and source

GPL-3.0 (ESP-SDR by ESPARGOS is GPL-3.0-or-later; see NOTICE in the source
for the ESPsoup additions and the C5VRX note). These binaries may only be
redistributed together with the corresponding source code: the source
commit is `{commit}`, built with ESP-IDF `{idf}`.

Build date {date}.
"""


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--version', required=True)
    parser.add_argument('--idf', default='', help='ESP-IDF commit used for the build')
    a = parser.parse_args()
    commit = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(['git', '-C', str(ROOT), 'status', '--porcelain', '--untracked-files=no'], capture_output=True, text=True).stdout.strip()
    if dirty:
        commit += ' (with uncommitted changes)'
    with tempfile.TemporaryDirectory() as tmp:
        manifest_path = export(a.build_dir, Path(tmp) / 'export', 'esp32c5',
                               f'ESP32-C5 ESPsoup {a.version}', a.version, allow_larger_flash=True)
        exported = json.loads(manifest_path.read_text())
        variant = exported['variants']['esp32c5']
        out = a.output
        out.mkdir(parents=True, exist_ok=False)
        parts = []
        for part in variant['parts']:
            data = (manifest_path.parent / 'esp32c5' / part['name']).read_bytes()
            base = part['name'].split('-', 1)[1]
            name = {'bootloader.bin': 'bootloader.bin', 'partition-table.bin': 'partition-table.bin'}.get(
                base, f'espsoup-firmware-{a.version}-esp32c5.bin')
            (out / name).write_bytes(data)
            assert sha256(data) == part['sha256']
            parts.append(dict(name=name, offset=part['offset'], size=part['size'], sha256=part['sha256'], md5=part['md5']))
    date = variant.get('build_date') or datetime.date.today().isoformat()
    manifest = dict(schema_version=1, name=f'ESPsoup firmware {a.version} (receive-only ESP-SDR for ESP32-C5, GPL-3.0)',
                    variants=dict(esp32c5=dict(
                        label=f'ESP32-C5 ESPsoup {a.version}', chip='ESP32-C5', target='esp32c5', version=a.version,
                        build_date=date, flash_size=variant['flash_size'], flash_size_policy='minimum',
                        flash_settings=variant['flash_settings'], source=dict(commit=commit, idf=a.idf), parts=parts)))
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', newline='\n')
    s = variant['flash_settings']
    flash_args = [f"--flash-mode {s['flash_mode']}", f"--flash-freq {s['flash_freq']}", f"--flash-size {s['flash_size']}"]
    flash_args += [f"{p['offset']:#x} {p['name']}" for p in parts]
    (out / 'flash_args').write_text('\n'.join(flash_args) + '\n', newline='\n')
    explicit = '    python -m esptool --chip esp32c5 -p PORT -b 921600 write-flash ' + ' '.join(flash_args)
    (out / 'README.md').write_text(README.format(version=a.version, flash_size=variant['flash_size'], explicit=explicit,
                                                 commit=commit, idf=a.idf or 'see firmware-targets.json', date=date), newline='\n')
    files = sorted(p.name for p in out.iterdir() if p.name != 'SHA256SUMS')
    (out / 'SHA256SUMS').write_text(''.join(f'{sha256((out / f).read_bytes())}  {f}\n' for f in files), newline='\n')
    print(out)


if __name__ == '__main__':
    main()
