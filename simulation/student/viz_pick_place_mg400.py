"""Looping animated MG400 pick-and-place in PyBullet or MuJoCo (--cycles limits runs)."""

import argparse
import os
import time
import sys
from functools import partial

os.environ.setdefault("MUJOCO_GL", "egl")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "runtime"))

from sim_mg400 import SimMG400, READY_POSE_MG400   # noqa: E402
from pick_place_common import run_visual_pick_place_cycles  # noqa: E402

PICK_X,  PICK_Y,  PICK_Z  = 380, -150, 20
PLACE_X, PLACE_Y, PLACE_Z = 380,  150, 20
LIFT       = 30
R          = 0
SUCTION_DO = 1

STEPS_LONG  = 40
STEPS_SHORT = 20
PAUSE       = 0.02


def _move(impl, x, y, z, r=0, steps=None):
    s = steps or STEPS_LONG
    impl.interpolate_to(x, y, z, r, steps=s, pause=PAUSE)


def go_home(impl):
    impl.interpolate_to(*READY_POSE_MG400, steps=STEPS_LONG, pause=PAUSE)


def pick_up(dashboard, move_impl):
    print("  Approach pick ...")
    _move(move_impl, PICK_X, PICK_Y, PICK_Z + LIFT)

    print("  Descend to pick ...")
    _move(move_impl, PICK_X, PICK_Y, PICK_Z, steps=STEPS_SHORT)

    print("  Suction ON ...")
    dashboard.ToolDO(SUCTION_DO, 1)
    time.sleep(0.3)

    print("  Lift ...")
    _move(move_impl, PICK_X, PICK_Y, PICK_Z + LIFT, steps=STEPS_SHORT)


def place_down(dashboard, move_impl):
    print("  Approach place ...")
    _move(move_impl, PLACE_X, PLACE_Y, PLACE_Z + LIFT)

    print("  Descend to place ...")
    _move(move_impl, PLACE_X, PLACE_Y, PLACE_Z, steps=STEPS_SHORT)

    print("  Suction OFF ...")
    dashboard.ToolDO(SUCTION_DO, 0)
    time.sleep(0.3)

    print("  Lift ...")
    _move(move_impl, PLACE_X, PLACE_Y, PLACE_Z + LIFT, steps=STEPS_SHORT)


def main():
    global PAUSE
    parser = argparse.ArgumentParser(description="MG400 animated pick-and-place visualization")
    parser.add_argument(
        "--backend", default=os.environ.get("DOBOT_SIM_BACKEND", "mujoco"),
        choices=["pybullet", "mujoco"],
        help="Simulation backend (default: mujoco)",
    )
    parser.add_argument(
        "--cycles", type=int, default=0,
        help="Number of pick-and-place cycles (0 = run until window closed)",
    )
    parser.add_argument(
        "--pause", type=float, default=PAUSE,
        help="Seconds per interpolation step (larger = slower, default 0.02)",
    )
    args = parser.parse_args()
    PAUSE = args.pause

    sim = SimMG400(backend=args.backend, gui=True)
    dashboard, move_api, feed = sim.connect()

    try:
        cycle = run_visual_pick_place_cycles(
            cycles=args.cycles,
            is_running=sim.is_running,
            go_home=partial(go_home, move_api),
            pick_up=partial(pick_up, dashboard, move_api),
            place_down=partial(place_down, dashboard, move_api),
        )
        print(f"\nCompleted {cycle} cycle(s).")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            dashboard.ToolDO(SUCTION_DO, 0)
        except Exception:
            pass
        sim.close()


if __name__ == "__main__":
    main()
