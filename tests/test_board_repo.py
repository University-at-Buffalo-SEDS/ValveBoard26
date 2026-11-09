"""Shared harnesses select this board's sources in a checkout of any name."""
from pathlib import Path
import tempfile
import unittest
from board_repo import ACTUATOR_BOARD, VALVE_BOARD, board_name

ROOT = Path(__file__).resolve().parents[1]


class BoardRepoTests(unittest.TestCase):
    def test_this_checkout_is_the_valve_board(self):
        self.assertEqual(board_name(ROOT), VALVE_BOARD)

    def test_checkout_directory_name_does_not_select_the_board(self):
        with tempfile.TemporaryDirectory() as tmp:
            for directory, ioc, expected in [
                ('vb', 'Valve_Board26.ioc', VALVE_BOARD),
                ('ActuatorBoard26', 'Valve_Board26.ioc', VALVE_BOARD),
                ('ValveBoard26', 'ActuationBoard.ioc', ACTUATOR_BOARD),
            ]:
                root = Path(tmp) / directory
                root.mkdir()
                (root / ioc).write_text('')
                self.assertEqual(board_name(root), expected, directory)
                (root / ioc).unlink()
                root.rmdir()

    def test_unknown_checkout_is_rejected_rather_than_guessed(self):
        with tempfile.TemporaryDirectory(prefix='ValveBoard26') as tmp:
            with self.assertRaises(RuntimeError):
                board_name(Path(tmp))


if __name__ == '__main__':
    unittest.main()
