"""The simulated tank pressure must sit below the board's overpressure relief.

The layout feeds ADC3 (the tank transducer) one fixed count. 2048 counts read
as 1320 psi, above the 850 psi relief threshold, so in every full-bay run the
Valve Board entered overpressure relief and sent AB_OVERPRESSURE_ABORT, and the
Actuator Board then refused every station FLIGHT_STATE from its Abort.
"""
import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def macro(path, name):
    text = (ROOT / path).read_text()
    return float(re.search(r"#define\s+" + name + r"\s+\(?([0-9.]+)f?\)?", text).group(1))


class SimTankPressureTests(unittest.TestCase):
    def test_layout_tank_pressure_is_below_relief(self):
        layout = json.loads((ROOT / "sim/board.json").read_text())
        adc3 = next(p for p in layout["peripherals"] if p.get("bus") == "adc3")
        driver = "Drivers/Pressure/pressure_transducer_driver.h"
        min_adc, max_adc = macro(driver, "PRESSURE_TRANSDUCER_MIN_ADC"), macro(driver, "PRESSURE_TRANSDUCER_MAX_ADC")
        min_psi, max_psi = macro(driver, "PRESSURE_TRANSDUCER_MIN_PRESSURE"), macro(driver, "PRESSURE_TRANSDUCER_MAX_PRESSURE")
        relief = macro("Core/Inc/overpressure.h", "OVERPRESSURE_RELIEF_PSI")
        worst = max(adc3["channel_samples"]) + adc3.get("noise_lsb", 0)
        psi = min_psi + (worst - min_adc) * (max_psi - min_psi) / (max_adc - min_adc)
        # Comfortably below relief, so noise or a model change cannot trip it.
        self.assertLess(psi, relief / 2, f"layout tank pressure {psi:.0f} psi")
        self.assertGreater(psi, 0.0)


if __name__ == "__main__":
    unittest.main()
