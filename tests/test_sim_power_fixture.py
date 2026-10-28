"""Keep the simulated power monitor wiring consistent with the populated board."""
import json
from pathlib import Path
import unittest


class PowerFixtureTests(unittest.TestCase):
    def test_voltage_and_current_sensors_have_external_i2c_pullups(self):
        layout = json.loads((Path(__file__).resolve().parents[1] / 'sim/board.json').read_text())
        pins = {(p['gpio'], p['pin']): p['initial'] for p in layout['board']['pins']}
        for pin in (8, 9):
            self.assertEqual(pins.get(('gpio', pin)), 'high')
        sensors = {p.get('bus'): p for p in layout['peripherals'] if p.get('model') == 'ltc2990'}
        self.assertEqual(set(sensors), {'i2c2@0x4c', 'i2c2@0x4d'})
        for sensor in sensors.values():
            self.assertEqual(sensor.get('bits', 12), 15)
            self.assertEqual(sensor.get('channels', 1), 4)
            self.assertEqual(len(sensor['channel_samples']), 4)
            self.assertTrue(all(0 <= code <= 0x7fff for code in sensor['channel_samples']))


if __name__ == '__main__':
    unittest.main()
