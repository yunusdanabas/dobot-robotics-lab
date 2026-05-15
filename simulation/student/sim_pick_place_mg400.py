"""Headless MG400 pick-and-place using SimMG400 (same motion pattern as mg400/08).

See simulation/student/README for backends and headless flags.
"""

import argparse
import os
import time
import sys
from functools import partial

# Suppress GLFW Wayland "no window position" warning on Ubuntu 24.04.
os.environ.setdefault("MUJOCO_GL", "egl")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "runtime"))

from sim_mg400 import SimMG400, SAFE_BOUNDS_MG400, READY_POSE_MG400   # noqa: E402
from pick_place_common import check_waypoints, run_single_pick_place    # noqa: E402

# ---------------------------------------------------------------------------
# Minimal motion helpers (clamping is already done inside MovJ / MovL)
# ---------------------------------------------------------------------------
JUMP_HEIGHT = 30
SAFE_BOUNDS = SAFE_BOUNDS_MG400
SUCTION_DO  = 1   # ToolDO index for suction pump


def safe_move(move_api, x, y, z, r, mode="J"):
    if mode == "L":
        move_api.MovL(x, y, z, r)
    else:
        move_api.MovJ(x, y, z, r)


def go_home(move_api):
    move_api.MovJ(*READY_POSE_MG400)
    move_api.Sync()

# ---------------------------------------------------------------------------
# User configuration - set these to match your table layout (mm)
# ---------------------------------------------------------------------------
PICK_X,  PICK_Y,  PICK_Z  = 280, -80, 20
PLACE_X, PLACE_Y, PLACE_Z = 280,  80, 20
LIFT = JUMP_HEIGHT
R    = 0

# ---------------------------------------------------------------------------
# Motion primitives - same pattern as the hardware pick-and-place script
# ---------------------------------------------------------------------------
def pick_up(dashboard, move_api):
    print("  Approach pick ...")
    safe_move(move_api, PICK_X, PICK_Y, PICK_Z + LIFT, R, mode="J")
    move_api.Sync()

    print("  Descend to pick ...")
    safe_move(move_api, PICK_X, PICK_Y, PICK_Z, R, mode="L")
    move_api.Sync()

    print("  Suction ON ...")
    dashboard.ToolDO(SUCTION_DO, 1)
    time.sleep(0.4)

    print("  Lift ...")
    safe_move(move_api, PICK_X, PICK_Y, PICK_Z + LIFT, R, mode="L")
    move_api.Sync()


def place_down(dashboard, move_api):
    print("  Approach place ...")
    safe_move(move_api, PLACE_X, PLACE_Y, PLACE_Z + LIFT, R, mode="J")
    move_api.Sync()

    print("  Descend to place ...")
    safe_move(move_api, PLACE_X, PLACE_Y, PLACE_Z, R, mode="L")
    move_api.Sync()

    print("  Suction OFF ...")
    dashboard.ToolDO(SUCTION_DO, 0)
    time.sleep(0.3)

    print("  Lift ...")
    safe_move(move_api, PLACE_X, PLACE_Y, PLACE_Z + LIFT, R, mode="L")
    move_api.Sync()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="MG400 pick-and-place simulation")
    parser.add_argument(
        "--backend", default=os.environ.get("DOBOT_SIM_BACKEND", "mujoco"),
        choices=["pybullet", "mujoco"],
        help="Simulation backend (default: mujoco)",
    )
    parser.add_argument("--no-viz", action="store_true", help="Disable simulator GUI")
    args = parser.parse_args()

    sim = SimMG400(backend=args.backend, gui=not args.no_viz)
    dashboard, move_api, feed = sim.connect()

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
            go_home=partial(go_home, move_api),
            pick_up=partial(pick_up, dashboard, move_api),
            place_down=partial(place_down, dashboard, move_api),
            home_message="Going to home ...",
            return_message="Returning to home ...",
        )
    finally:
        try:
            dashboard.ToolDO(SUCTION_DO, 0)
        except Exception:
            pass
        sim.close()


if __name__ == "__main__":
    main()
