"""Small command-line simulations for the pure robot decision logic."""

from __future__ import annotations

try:
    from algorithm.maze_logic import (
        GreenStopDemoController,
        RightHandMazeController,
        SensorFrame,
        Signal,
    )
except ModuleNotFoundError:
    from maze_logic import GreenStopDemoController, RightHandMazeController, SensorFrame, Signal


def run_maze_smoke() -> list[str]:
    controller = RightHandMazeController()
    frames = [
        SensorFrame(front_cm=30, left_cm=10, right_cm=10, signal=Signal.UNKNOWN),
        SensorFrame(front_cm=30, left_cm=10, right_cm=30, signal=Signal.UNKNOWN),
        SensorFrame(front_cm=8, left_cm=30, right_cm=8, signal=Signal.UNKNOWN),
        SensorFrame(front_cm=8, left_cm=8, right_cm=8, signal=Signal.UNKNOWN),
        SensorFrame(front_cm=30, left_cm=10, right_cm=10, signal=Signal.STOP),
        SensorFrame(front_cm=30, left_cm=10, right_cm=10, signal=Signal.UNKNOWN),
        SensorFrame(front_cm=30, left_cm=10, right_cm=10, signal=Signal.GO),
    ]
    lines = []
    for idx, frame in enumerate(frames, start=1):
        d = controller.decide(frame)
        lines.append(
            f"{idx:02d} maze: state={d.state:<12} action={d.action.value:<10} "
            f"L={d.left_pwm:>5.1f} R={d.right_pwm:>5.1f} reason={d.reason}"
        )
    return lines


def run_demo_smoke() -> list[str]:
    controller = GreenStopDemoController()
    lines = []
    for idx, signal in enumerate([Signal.UNKNOWN, Signal.STOP, Signal.GO, Signal.UNKNOWN], start=1):
        d = controller.decide(signal)
        lines.append(
            f"{idx:02d} demo: signal={signal.value:<7} state={d.state:<18} "
            f"action={d.action.value:<10} reason={d.reason}"
        )
    return lines


def main() -> None:
    for line in run_maze_smoke():
        print(line)
    for line in run_demo_smoke():
        print(line)


if __name__ == "__main__":
    main()
