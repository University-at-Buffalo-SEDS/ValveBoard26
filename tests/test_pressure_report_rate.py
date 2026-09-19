import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class PressureReportRate(unittest.TestCase):
    def compile_rate(self, rate=None):
        compiler = shutil.which("cc")
        self.assertIsNotNone(compiler, "host C compiler required for rate tests")
        source = '''
#include "telemetry_rates.h"
#include <assert.h>
int main(void) {
    assert(valve_pressure_sleep_ticks(0) == VALVE_PRESSURE_REPORT_TICKS);
    assert(valve_pressure_sleep_ticks(2) + 2 == VALVE_PRESSURE_REPORT_TICKS);
    assert(valve_pressure_sleep_ticks(VALVE_PRESSURE_REPORT_TICKS) == 1);
    assert(valve_pressure_sleep_ticks(1000) == 1);
#if VALVE_PRESSURE_REPORT_HZ == 30
    assert(VALVE_PRESSURE_REPORT_TICKS == 33);
#elif VALVE_PRESSURE_REPORT_HZ == 50
    assert(VALVE_PRESSURE_REPORT_TICKS == 20);
#endif
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "rate-test"
            command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                       "-DTX_TIMER_TICKS_PER_SECOND=1000", "-I", str(ROOT / "Core/Inc")]
            if rate is not None:
                command.append(f"-DVALVE_PRESSURE_REPORT_HZ={rate}")
            result = subprocess.run(command + ["-x", "c", "-", "-o", str(binary)],
                                    input=source, text=True, capture_output=True)
            if rate in (0, 101):
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("100 Hz ADC sample rate", result.stderr)
            else:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(binary)], check=True)

    def test_default_and_override_cadence_and_overrun_yield(self):
        for rate in (None, 50):
            with self.subTest(rate=rate):
                self.compile_rate(rate)

    def test_invalid_sample_rates_are_rejected(self):
        for rate in (0, 101):
            with self.subTest(rate=rate):
                self.compile_rate(rate)

    def test_pressure_rate_is_independent_of_one_hz_power_reports(self):
        source = (ROOT / "Core/Src/data_acq_thread.c").read_text()
        self.assertIn("#define DATA_ACQ_PRESSURE_PERIOD_TICKS VALVE_PRESSURE_REPORT_TICKS", source)
        self.assertIn("#define DATA_ACQ_REPORT_PERIOD_TICKS ((ULONG)TX_TIMER_TICKS_PER_SECOND)", source)
        self.assertIn("tx_thread_sleep(valve_pressure_sleep_ticks", source)
