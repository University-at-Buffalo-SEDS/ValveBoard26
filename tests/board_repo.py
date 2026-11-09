"""Name the board a checkout holds from its sources, not its directory name.

The host harnesses are shared with ActuatorBoard26 and pick board-specific
sources and stubs from this name. A worktree or clone can live in any
directory, so the directory name would select the wrong board's harness.
"""
from pathlib import Path

VALVE_BOARD = 'ValveBoard26'
ACTUATOR_BOARD = 'ActuatorBoard26'


def board_name(root: Path) -> str:
    root = Path(root)
    if (root / 'Valve_Board26.ioc').is_file():
        return VALVE_BOARD
    if (root / 'ActuationBoard.ioc').is_file():
        return ACTUATOR_BOARD
    raise RuntimeError(f'{root} is neither a ValveBoard26 nor an ActuatorBoard26 checkout')
