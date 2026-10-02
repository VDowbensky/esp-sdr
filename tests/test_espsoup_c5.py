# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 ESPsoup contributors / Peter Holzhauser
"""Host checks of the ESPsoup ESP32-C5 DSP (CSPEC, VFM, GFSK); no RF hardware is used.

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

    @unittest.skipUnless(shutil.which('node'), 'node unavailable')
    def test_vfm_and_gfsk_demodulators(self):
        with tempfile.TemporaryDirectory() as tmp:
            env = dict(os.environ, DEMOD_HOST=self.build(tmp, 'demod_host.c', 'demod_host'), ESPSOUP_TEST_DIR=tmp)
            for script in ['test_vfm.mjs', 'test_gfsk.mjs']:
                with self.subTest(script=script):
                    run = subprocess.run(['node', str(HERE / script)], env=env, capture_output=True, text=True)
                    self.assertEqual(run.returncode, 0, run.stdout[-4000:] + run.stderr[-2000:])
                    self.assertNotIn('FAIL ', run.stdout)


class EspsoupC5Tables(unittest.TestCase):
    def test_generated_tables_are_current(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / 'demod_tab.h'
            subprocess.run([sys.executable, str(HERE / 'gen_tables.py'), str(out)], check=True, capture_output=True)
            self.assertEqual(out.read_text(), (FAMILY / 'demod_tab.h').read_text())


if __name__ == '__main__':
    unittest.main()
