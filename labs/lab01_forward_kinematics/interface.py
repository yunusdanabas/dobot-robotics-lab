"""
Lab 01 - Forward Kinematics (ME403, Sabanci University).

Run this file. Implement run(robot) in myCode.py.

Select magician or mg400 with DOBOT_ROBOT_TYPE. 
For env vars, FK notes, and tooling, see student_api.md; 
For joint frames and conventions, see myCode.py.

Simulation only: after tasks finish, keep the 3-D viewer open with
DOBOT_SIM_HOLD=1 (waits for Enter in a real terminal) or
DOBOT_SIM_HOLD_SECS=<seconds> (sleep, then close; works without a TTY).
"""

from __future__ import annotations

import os
import sys
import time

import myCode
import utils as U


def _pause_before_teardown(robot: U.RobotSession) -> None:
    """Optional delay so the MuJoCo/PyBullet window stays visible."""
    if not getattr(robot, "simulation", False):
        return
    secs_raw = os.environ.get("DOBOT_SIM_HOLD_SECS", "").strip()
    if secs_raw:
        try:
            hold_secs = max(0.0, float(secs_raw))
        except ValueError:
            print(f"[hold] Ignoring invalid DOBOT_SIM_HOLD_SECS={secs_raw!r}.")
            return
        time.sleep(hold_secs)
        return
    hold = os.environ.get("DOBOT_SIM_HOLD", "").strip().lower()
    if hold not in {"1", "true", "yes", "on"}:
        return
    if sys.stdin.isatty():
        input("Press Enter to close the simulation and release the window... ")
    else:
        time.sleep(15.0)


def main() -> None:
    robot = U.setup()
    try:
        myCode.run(robot)
        _pause_before_teardown(robot)
    finally:
        U.teardown(robot)
        print("Done.")


if __name__ == "__main__":
    main()
