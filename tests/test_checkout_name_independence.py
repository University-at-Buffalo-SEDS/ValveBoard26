"""Tests must not depend on the name of the checkout directory.

Two heartbeat-owner tests shared with the Actuator Board picked their
variant from the checkout directory name, so a clone or worktree under
another name could run the wrong board's variant. Board identity comes
from board_identity.BOARD_REPOSITORY, read from sim/board.json.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CheckoutNameIndependenceTests(unittest.TestCase):
    def test_no_test_selects_a_board_by_directory_name(self):
        pattern = re.compile(r"(ROOT|root)\.name\b[^\n]*(ActuatorBoard26|ValveBoard26)|b=root\.name")
        offenders = [path.name for path in sorted((ROOT / 'tests').glob('*.py'))
                     if path.name != Path(__file__).name and pattern.search(path.read_text())]
        self.assertEqual(offenders, [])

    def test_board_identity_follows_the_layout(self):
        from board_identity import BOARD_REPOSITORY
        self.assertEqual(BOARD_REPOSITORY, 'ValveBoard26')


if __name__ == '__main__':
    unittest.main()
