"""Actual capture/publisher finite-retention invariants; no fake SDK claim."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'tests/native/pressure_retention'


class PressureRetentionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.temporary.name) / 'worker'
        source = (FIXTURE / 'worker.c').read_text()
        conversion = (ROOT / 'Drivers/Pressure/pressure_transducer_driver.c').read_text()
        conversion = conversion[conversion.index('float pressureTransducerConvertRaw('):
                                conversion.index('static HAL_StatusTypeDef pressure_configure_channel')]
        source = source.replace('/* ACTUAL_CONVERSION_FUNCTION */', conversion)
        command = ['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                   '-fsanitize=address,undefined', '-I', str(FIXTURE / 'stubs'),
                   '-I', str(FIXTURE), '-I', str(ROOT / 'Core/Inc'),
                   '-I', str(ROOT / 'Core/Src'), '-I', str(ROOT / 'Drivers/Pressure'),
                   '-x', 'c', '-', '-o', str(cls.exe)]
        result = subprocess.run(command, input=source, text=True, capture_output=True)
        if result.returncode:
            cls.temporary.cleanup()
            raise AssertionError(result.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_case(self, prefill, phase, blackout, wrap=0, terminal=0, calibration=0):
        result = subprocess.run([str(self.exe), *map(str, [prefill, phase, blackout,
                                                        wrap, terminal, calibration])],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_finite_deficit_phases_and_clock_wrap(self):
        for prefill in range(1, 5):
            for phase in range(16):
                with self.subTest(prefill=prefill, phase=phase):
                    self.run_case(prefill, phase, 402)
        self.run_case(4, 15, 402, wrap=1)

    def test_prepared_bytes_survive_calibration_change_and_duplicate_init(self):
        self.run_case(4, 0, 402, calibration=1)

    def test_terminal_non_io_discard_is_counted_once(self):
        self.run_case(4, 0, 402, terminal=1)

    def test_finite_and_persistent_overload_keeps_real_overflow_visible(self):
        self.run_case(4, 0, 2000)
        self.run_case(4, 0, 10000)
