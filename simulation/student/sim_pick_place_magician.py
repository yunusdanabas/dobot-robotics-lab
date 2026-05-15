"""Headless Magician pick-and-place using SimDobot (same motion pattern as magician/08).

Pick/place mm coordinates match the hardware script; use --backend / --no-viz / DOBOT_SIM_BACKEND
as in simulation/student/README.
"""

import argparse
import os
import sys
import time
from functools import partial

# Suppress GLFW Wayland "no window position" warning on Ubuntu 24.04.
# EGL bypasses GLFW entirely; setdefault lets users override with osmesa.
os.environ.setdefault("MUJOCO_GL", "egl")

# Make magician/utils importable (sim script lives one level below repo root).
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "runtime"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "robots", "magician"))

from sim_dobot import SimDobot                                    # noqa: E402
from utils import safe_move, go_home, SAFE_BOUNDS, JUMP_HEIGHT   # noqa: E402
from pick_place_common import check_waypoints, run_single_pick_place  # noqa: E402

# ---------------------------------------------------------------------------
# User configuration - set these to match your table layout (mm)
# ---------------------------------------------------------------------------
PICK_X,  PICK_Y,  PICK_Z  = 220, -60, 30
PLACE_X, PLACE_Y, PLACE_Z = 220,  60, 30
LIFT = JUMP_HEIGHT   # mm above pick/place Z for safe travel
R    = 0             # end-effector rotation (deg)

# ---------------------------------------------------------------------------
# Motion primitives - same pattern as the hardware pick-and-place script
# ---------------------------------------------------------------------------
def pick_up(bot):
    print("  Approach pick ...")
    safe_move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, R)

    print("  Descend to pick ...")
    safe_move(bot, PICK_X, PICK_Y, PICK_Z, R)

    print("  Suction ON ...")
    bot.suck(True)
    time.sleep(0.4)

    print("  Lift ...")
    safe_move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, R)


def place_down(bot):
    print("  Approach place ...")
    safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, R)

    print("  Descend to place ...")
    safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z, R)

    print("  Suction OFF ...")
    bot.suck(False)
    time.sleep(0.3)

    print("  Lift ...")
    safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, R)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Magician pick-and-place simulation")
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
    parser.add_argument("--no-viz", action="store_true", help="Disable simulator GUI")
    args = parser.parse_args()

    bot = SimDobot(backend=args.backend, gui=not args.no_viz, tool=args.ee_mode)

    try:
        check_waypoints(
            bounds=SAFE_BOUNDS,
            points=[
                ("PICK", PICK_X, PICK_Y, PICK_Z),
                ("PICK_APPROACH", PICK_X, PICK_Y, PICK_Z + LIFT),
                ("PLACE", PLACE_X, PLACE_Y, PLACE_Z),
                ("PLACE_APPROACH", PLACE_X, PLACE_Y, PLACE_Z + LIFT),
            ],
        )

        run_single_pick_place(
            go_home=partial(go_home, bot),
            pick_up=partial(pick_up, bot),
            place_down=partial(place_down, bot),
        )
    finally:
        try:
            bot.suck(False)
        except Exception:
            pass
        bot.close()


if __name__ == "__main__":
    main()
