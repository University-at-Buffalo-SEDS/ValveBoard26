import json
from pathlib import Path
import unittest


class SimAdcContract(unittest.TestCase):
    def test_network_qualification_keeps_onboard_adcs_connected(self):
        layout = json.loads((Path(__file__).resolve().parents[1] / "sim/board.json").read_text())
        for peripheral in layout["peripherals"]:
            if peripheral["type"] == "adc":
                with self.subTest(peripheral=peripheral["name"]):
                    self.assertFalse(peripheral.get("failure_every", 0))
                    self.assertFalse(peripheral.get("disconnect_after", 0))
