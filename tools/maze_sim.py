"""Virtual maze simulator — end-to-end algorithm verification.

Runs the *production* `WallFollowerSM` against synthetic mazes with
ray-cast ultrasonic readings and differential-drive physics. Verifies
that the algorithm reaches the exit without scraping walls — final
sanity check before we commit Pi hardware time.

Mazes:
  default — 100x80 L-corridor: straight driving, one right opening that
            requires the wall-reacquire turn, one anticipated corner.
  uturn   — 140x100 with a spine wall: the followed right wall ENDS and
            the car must wrap ~180 deg around the thin wall end. This is
            the #1 field-failure case (2026-06-10 runs).

Noise: --none-rate p drops each sensor reading to None with probability
p per tick (seeded, reproducible) — models the buried-sensor dropout
bursts seen on the real car.

What this does NOT validate:
  - Real ultrasonic noise spectrum / 우드락 reflection bias
  - Real motor asymmetry / drift / wheel slip
  - Camera latency

Usage:
    python tools/maze_sim.py                          # default maze
    python tools/maze_sim.py --maze uturn             # U-turn failure case
    python tools/maze_sim.py --none-rate 0.25         # with dropout noise
    python tools/maze_sim.py --verbose --render
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from algorithm.wall_follower_sm import WallFollowerSM  # noqa: E402
from logs.trace import tracer  # noqa: E402


# ---------------------------------------------------------------------- #
# Maze definitions (cm units). Walls are axis-aligned rectangles
# (x_min, y_min, x_max, y_max).
# ---------------------------------------------------------------------- #
MAZES: dict[str, dict] = {
    # 100x80 with a horizontal partition -> L-shaped corridor. The
    # partition END at x=70 is a right-opening the car must take.
    "default": {
        "walls": [
            (0, 0, 100, 1),       # south
            (0, 79, 100, 80),     # north
            (0, 0, 1, 80),        # west
            (99, 0, 100, 80),     # east
            (0, 40, 70, 41),      # partition; gap on the east side
        ],
        "start": (15.0, 60.0, 0.0),          # top corridor, heading east
        "exit_region": (30.0, 17.0, 50.0, 28.0),
        "size": (100, 80),
    },
    # 140x100 with a spine from the west wall. Start in the NORTH corridor
    # heading east with the spine as the followed right wall; at x=100 the
    # spine ENDS and the car must wrap ~180 deg around the 4cm-thin end
    # into the south corridor — the U-turn failure case.
    "uturn": {
        "walls": [
            (0, 0, 140, 1),       # south
            (0, 99, 140, 100),    # north
            (0, 0, 1, 100),       # west
            (139, 0, 140, 100),   # east
            (0, 48, 100, 52),     # spine from the west wall, ends at x=100
        ],
        "start": (10.0, 72.0, 0.0),          # north corridor, heading east
        "exit_region": (2.0, 2.0, 28.0, 46.0),  # south corridor, west end
        "size": (140, 100),
    },
}

# Car geometry & physics.
CAR_RADIUS_CM = 9.0          # half the 18cm chassis-with-wheels footprint
WHEELBASE_CM = 15.0
PWM_TO_CMS = 0.5             # matches placeholder SPEED_CM_PER_SEC_AT_50 / 50

SENSOR_MAX_CM = 400.0

SIM_DT = 0.1                 # 10 Hz, matches main.py
MAX_TICKS = 600              # 60 sim seconds


def ray_rect_distance(
    ox: float, oy: float, dx: float, dy: float,
    x_min: float, y_min: float, x_max: float, y_max: float,
) -> float:
    """Distance from (ox, oy) along (dx, dy) to first contact with rect."""
    if abs(dx) < 1e-12:
        if ox < x_min or ox > x_max:
            return math.inf
        tx_in, tx_out = -math.inf, math.inf
    else:
        t1 = (x_min - ox) / dx
        t2 = (x_max - ox) / dx
        tx_in, tx_out = (t1, t2) if t1 < t2 else (t2, t1)

    if abs(dy) < 1e-12:
        if oy < y_min or oy > y_max:
            return math.inf
        ty_in, ty_out = -math.inf, math.inf
    else:
        t1 = (y_min - oy) / dy
        t2 = (y_max - oy) / dy
        ty_in, ty_out = (t1, t2) if t1 < t2 else (t2, t1)

    t_enter = max(tx_in, ty_in)
    t_exit = min(tx_out, ty_out)
    if t_enter > t_exit or t_exit < 0:
        return math.inf
    return max(t_enter, 0.0)


def sensor_reading(
    walls: list[tuple[float, float, float, float]],
    x: float, y: float, theta: float,
) -> float | None:
    dx, dy = math.cos(theta), math.sin(theta)
    best = math.inf
    for wall in walls:
        d = ray_rect_distance(x, y, dx, dy, *wall)
        if d < best:
            best = d
    if best >= SENSOR_MAX_CM:
        return None
    return best


def check_collision(
    walls: list[tuple[float, float, float, float]], x: float, y: float
) -> bool:
    for x_min, y_min, x_max, y_max in walls:
        cx = max(x_min, min(x, x_max))
        cy = max(y_min, min(y, y_max))
        if (cx - x) ** 2 + (cy - y) ** 2 < CAR_RADIUS_CM ** 2:
            return True
    return False


def in_exit(exit_region: tuple[float, float, float, float], x: float, y: float) -> bool:
    return (
        exit_region[0] <= x <= exit_region[2]
        and exit_region[1] <= y <= exit_region[3]
    )


def update_car(
    x: float, y: float, theta: float,
    left_pwm: float, right_pwm: float, dt: float,
) -> tuple[float, float, float]:
    v_left = left_pwm * PWM_TO_CMS
    v_right = right_pwm * PWM_TO_CMS
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / WHEELBASE_CM
    new_x = x + v * math.cos(theta) * dt
    new_y = y + v * math.sin(theta) * dt
    new_theta = theta + omega * dt
    return new_x, new_y, new_theta


def execute_cmd(
    cmd, x: float, y: float, theta: float, dt: float,
) -> tuple[float, float, float]:
    action = cmd.action
    speed = cmd.linear_speed
    if action == "stop":
        l, r = 0.0, 0.0
    elif action == "forward":
        l, r = speed, speed
    elif action == "backward":
        l, r = -speed, -speed
    elif action == "arc":
        c = max(-1.0, min(1.0, cmd.curvature))
        if c >= 0:
            l, r = speed * (1.0 - c), speed
        else:
            l, r = speed, speed * (1.0 + c)
    elif action == "pivot_right":
        l, r = speed, -speed
    elif action == "pivot_left":
        l, r = -speed, speed
    else:
        l, r = 0.0, 0.0
    return update_car(x, y, theta, l, r, dt)


def render_trajectory(maze: dict, trajectory: list[tuple[float, float]]) -> None:
    """ASCII map: '#' walls, '·' trajectory, 'S' start, 'E' exit."""
    cell_cm = 2.0
    width, height = maze["size"]
    width_cells = int(width / cell_cm)
    height_cells = int(height / cell_cm)
    grid = [[" "] * width_cells for _ in range(height_cells)]

    for x_min, y_min, x_max, y_max in maze["walls"]:
        cx_min = int(x_min / cell_cm)
        cx_max = max(cx_min + 1, int(x_max / cell_cm))
        cy_min = int(y_min / cell_cm)
        cy_max = max(cy_min + 1, int(y_max / cell_cm))
        for r in range(cy_min, min(cy_max, height_cells)):
            for c in range(cx_min, min(cx_max, width_cells)):
                grid[r][c] = "#"

    for x, y in trajectory:
        c = int(x / cell_cm)
        r = int(y / cell_cm)
        if 0 <= r < height_cells and 0 <= c < width_cells and grid[r][c] == " ":
            grid[r][c] = "·"

    sx, sy, _ = maze["start"]
    sc, sr = int(sx / cell_cm), int(sy / cell_cm)
    if 0 <= sr < height_cells and 0 <= sc < width_cells:
        grid[sr][sc] = "S"
    ex_region = maze["exit_region"]
    ex_cx = int((ex_region[0] + ex_region[2]) / 2 / cell_cm)
    ex_cy = int((ex_region[1] + ex_region[3]) / 2 / cell_cm)
    if 0 <= ex_cy < height_cells and 0 <= ex_cx < width_cells:
        grid[ex_cy][ex_cx] = "E"

    print()
    print("  Maze map (north on top, S=start, E=exit, ·=trajectory):")
    print()
    for row in reversed(grid):
        print("    " + "".join(row))
    print()


def run_sim(
    maze_name: str,
    none_rate: float,
    seed: int,
    max_ticks: int,
    verbose: bool = False,
    render: bool = False,
    trace_name: str | None = None,
) -> tuple[str, int]:
    maze = MAZES[maze_name]
    walls = maze["walls"]
    exit_region = maze["exit_region"]

    sm = WallFollowerSM()
    tracer.start(trace_name or f"maze_sim_{maze_name}")
    rng = random.Random(seed)

    x, y, theta = maze["start"]
    trajectory: list[tuple[float, float]] = [(x, y)]
    outcome: tuple[str, int] | None = None

    def noisy(v: float | None) -> float | None:
        if none_rate > 0.0 and rng.random() < none_rate:
            return None
        return v

    print(
        f"[SIM] maze={maze_name} none_rate={none_rate} seed={seed} "
        f"start=({x:.1f},{y:.1f}) heading {math.degrees(theta):.0f}°"
    )
    print(f"[SIM] exit region {exit_region}")

    for tick in range(max_ticks):
        f = noisy(sensor_reading(walls, x, y, theta))
        l = noisy(sensor_reading(walls, x, y, theta + math.pi / 4))
        r = noisy(sensor_reading(walls, x, y, theta - math.pi / 4))
        signal = "UNKNOWN"

        cmd = sm.step(f, l, r, signal)
        x, y, theta = execute_cmd(cmd, x, y, theta, SIM_DT)
        trajectory.append((x, y))

        if verbose and tick % 5 == 0:
            f_s = f"{f:5.1f}" if f is not None else "  inf"
            l_s = f"{l:5.1f}" if l is not None else "  inf"
            r_s = f"{r:5.1f}" if r is not None else "  inf"
            print(
                f"  t={tick * SIM_DT:5.1f}  "
                f"pos=({x:5.1f},{y:5.1f}) θ={math.degrees(theta):+7.1f}°  "
                f"f={f_s}  l={l_s}  r={r_s}  "
                f"state={sm.state:<14} "
                f"{cmd.action} c={cmd.curvature:+.2f} s={cmd.linear_speed:.0f}"
            )

        if check_collision(walls, x, y):
            outcome = ("COLLISION", tick)
            break
        if in_exit(exit_region, x, y):
            outcome = ("EXIT_REACHED", tick)
            break

    if outcome is None:
        outcome = ("TIMEOUT", max_ticks)

    tracer.info("sim_done", maze=maze_name, none_rate=none_rate,
                outcome=outcome[0], ticks=outcome[1])
    tracer.stop()

    sim_seconds = outcome[1] * SIM_DT
    print()
    print("=" * 60)
    print("  SIM RESULT")
    print("=" * 60)
    print(f"  maze           : {maze_name}  (none_rate={none_rate}, seed={seed})")
    print(f"  outcome        : {outcome[0]}")
    print(f"  sim duration   : {sim_seconds:.1f}s  ({outcome[1]} ticks)")
    print(f"  final position : ({x:.1f}, {y:.1f})  θ={math.degrees(theta):+.1f}°")
    print(f"  final state    : {sm.state}")
    print(f"  distance trav. : {_path_length(trajectory):.1f} cm")
    print("=" * 60)

    if render:
        render_trajectory(maze, trajectory)

    return outcome


def _path_length(traj: list[tuple[float, float]]) -> float:
    total = 0.0
    for (x1, y1), (x2, y2) in zip(traj, traj[1:]):
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def main() -> int:
    ap = argparse.ArgumentParser(description="Virtual maze simulator")
    ap.add_argument("--maze", choices=sorted(MAZES.keys()), default="default")
    ap.add_argument("--none-rate", type=float, default=0.0,
                    help="per-sensor per-tick dropout probability")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--max-ticks", type=int, default=MAX_TICKS)
    args = ap.parse_args()

    outcome = run_sim(
        args.maze, args.none_rate, args.seed, args.max_ticks,
        verbose=args.verbose, render=args.render,
    )
    return 0 if outcome[0] == "EXIT_REACHED" else 1


if __name__ == "__main__":
    sys.exit(main())
