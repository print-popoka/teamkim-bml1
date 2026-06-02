"""High-level decision logic for the maze robot.

Two distinct entry points live in this package — keep the distinction
clear when reading code:

  - ``WallFollowerSM`` / ``HighLevelCommand`` (``wall_follower_sm.py``)
    → PRODUCTION. ``main.py`` runs this; it delegates to the smooth-PD
    ``WallFollowController`` in ``control/wall_follow.py``.

  - ``RightHandMazeController`` / ``GreenStopDemoController``
    (``maze_logic.py``) → OFFLINE SIMULATOR. Used only by
    ``algorithm/simulate.py`` and ``tests/test_maze_logic.py`` to
    validate discrete-decision invariants without hardware deps.
"""

from .maze_logic import (
    Action,
    Decision,
    GreenStopDemoController,
    RightHandMazeController,
    SensorFrame,
    Signal,
)
from .wall_follower_sm import HighLevelCommand, WallFollowerSM

__all__ = [
    # Production state machine (used by main.py)
    "HighLevelCommand",
    "WallFollowerSM",
    # Offline simulator types (used by simulate.py / tests)
    "Action",
    "Decision",
    "GreenStopDemoController",
    "RightHandMazeController",
    "SensorFrame",
    "Signal",
]
