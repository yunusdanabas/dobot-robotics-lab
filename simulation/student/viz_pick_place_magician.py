"""Looping animated Magician pick-and-place in MuJoCo or PyBullet (viewer until closed or --cycles)."""

import argparse
import os
import sys
import time
from functools import partial

os.environ.setdefault("MUJOCO_GL", "egl")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "runtime"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "robots", "magician"))

from sim_dobot import SimDobot                                    # noqa: E402
from utils import SAFE_BOUNDS, JUMP_HEIGHT                       # noqa: E402
from pick_place_common import run_visual_pick_place_cycles       # noqa: E402

PICK_X,  PICK_Y,  PICK_Z  = 220, -60, 30
PLACE_X, PLACE_Y, PLACE_Z = 220,  60, 30
LIFT = JUMP_HEIGHT
R    = 0

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0]

STEPS_LONG  = 40   # joint moves - longer travel
STEPS_SHORT = 20   # short vertical dips
PAUSE       = 0.02 # seconds per interpolation step


def _move(bot, x, y, z, r=0, steps=None):
    s = steps or STEPS_LONG
    bot.interpolate_to(x, y, z, r, steps=s, pause=PAUSE)


def go_home(bot):
    bot.interpolate_joints_to(HOME_JOINTS, steps=STEPS_LONG, pause=PAUSE)


def pick_up(bot):
    print("  Approach pick ...")
    _move(bot, PICK_X, PICK_Y, PICK_Z + LIFT)

    print("  Descend to pick ...")
    _move(bot, PICK_X, PICK_Y, PICK_Z, steps=STEPS_SHORT)

    print("  Suction ON ...")
    bot.suck(True)
    time.sleep(0.3)

    print("  Lift ...")
    _move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, steps=STEPS_SHORT)


def place_down(bot):
    print("  Approach place ...")
    _move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT)

    print("  Descend to place ...")
    _move(bot, PLACE_X, PLACE_Y, PLACE_Z, steps=STEPS_SHORT)

    print("  Suction OFF ...")
    bot.suck(False)
    time.sleep(0.3)

    print("  Lift ...")
    _move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, steps=STEPS_SHORT)


def main():
    global PAUSE
    parser = argparse.ArgumentParser(description="Magician animated pick-and-place visualization")
    parser.add_argument(
        "--backend", default=os.environ.get("DOBOT_SIM_BACKEND", "mujoco"),
        choices=["pybullet", "mujoco"],
        help="Simulation backend (default: mujoco)",
    )
    parser.add_argument(
        "--ee-mode",
        choices=["none", "motor", "suction"],
        default=os.environ.get("DOBOT_EE", "none"),
        help=(
            "End-effector mode (default: none). "
            "motor=+60mm X; suction=+60mm X -70mm Z (physical cup tip)."
        ),
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

    bot = SimDobot(backend=args.backend, gui=True, tool=args.ee_mode)

    try:
        cycle = run_visual_pick_place_cycles(
            cycles=args.cycles,
            is_running=bot.is_running,
            go_home=partial(go_home, bot),
            pick_up=partial(pick_up, bot),
            place_down=partial(place_down, bot),
        )
        print(f"\nCompleted {cycle} cycle(s).")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            bot.suck(False)
        except Exception:
            pass
        bot.close()


if __name__ == "__main__":
    main()
