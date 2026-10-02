# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Host checks of the ESPsoup ESP32-C5 DSP (CSPEC); no RF hardware is used.

The firmware DSP headers are integer-only and compile unchanged on the host,
so these results are bit-exact with the device (see docs/espsoup-c5.md)."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
FAMILY = ROOT / 'main/families/c5_c6_c61'
HERE = ROOT / 'tests/espsoup'


@unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
class EspsoupC5Dsp(unittest.TestCase):
    def build(self, tmp, source, name):
        exe = Path(tmp) / name
        subprocess.run(['cc', '-std=gnu11', '-O2', '-I' + str(FAMILY), str(HERE / source), '-o', str(exe), '-lm'], check=True)
        return str(exe) + ('.exe' if os.name == 'nt' and not exe.exists() else '')

    def test_cspec_matches_reference_fft(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = subprocess.run([self.build(tmp, 'cspec_host_test.c', 'cspec')], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn('PASS', run.stdout)




if __name__ == '__main__':
    unittest.main()
