"""Board identity for shared Actuator/Valve tests.

Read from the layout that travels with the sources (sim/board.json), never
from the checkout directory name: a worktree or clone under another name
must still select this board's variant of a shared test.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_LAYOUT = json.loads((ROOT / 'sim/board.json').read_text())['name']
BOARD_REPOSITORY = {'ActuationBoard': 'ActuatorBoard26',
                    'Valve_Board26': 'ValveBoard26'}.get(_LAYOUT, ROOT.name)
