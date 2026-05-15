"""Lab 01 - edit this file. interface.py calls run(robot).

Body-frame joints q = [q1, q2, q3, q4] (degrees).

Magician matches firmware GetPose: q3 is the world forearm angle (0 = horizontal).
This lab's FK uses q1, q2, q3 only. Home (0,0,0,0) gives TCP (147, 0, 135) mm
with tool none.

MG400: firmware J3 = J2 + q3. In fk_predict use U.MG400_API_* so results agree with
sim and GetPose (student_api.md), not planar ROBOT_MODELS lengths alone.
"""

from __future__ import annotations

import math

import utils as U


def fk_predict(
    q: list[float],
    L1: float,
    L2: float,
    Z_base: float,
    robot_type: str,
) -> tuple[float, float, float]:
    """Return predicted (x, y, z) in mm from body-frame q = [q1..q4] (degrees).

    Magician: use L1 and L2 only; Z_base is unused (shoulder FK).

    MG400: ignore L1, L2, Z_base; use U.MG400_API_* so the prediction matches sim and
    GetPose (student_api.md). Planar teaching lengths stay in ROBOT_MODELS.
    """
    # TODO: Implement forward kinematics prediction.
    # Hint: Convert degrees to radians before sin/cos.
    # Magician: reach = L1*sin(q2) + L2*cos(q3), z = L1*cos(q2) - L2*sin(q3)
    # MG400: use U.MG400_API_* constants (see student_api.md)
    raise NotImplementedError("TODO: implement fk_predict()")


def run(robot: U.RobotSession) -> None:
    """Tasks 0-3. robot is the session from U.setup()."""
    L1 = U.L1
    L2 = U.L2
    zb = U.Z_base
    rtype = robot.type

    # Task 0: Home pose
    # TODO: Move to joint home q=(0,0,0,0) if not already there.
    # TODO: Print the actual home pose from get_pose().
    # TODO: Compare to fk_predict() at home.

    # Task 1: Single configuration
    # TODO: Choose a joint configuration, move there.
    # TODO: Print predicted pose from fk_predict() and actual get_pose().
    # TODO: Print XY and Z errors.

    # Task 2: Multi-step trajectory
    # TODO: Build a list of 3-5 joint configurations.
    # TODO: For each: move_joints(), then get_pose(), then print pose.

    # Task 3: FK vs actual along the trajectory
    # TODO: For each q: fk_predict(), get_pose(), XY error = hypot(xa-xp, ya-yp),
    #       Z error = abs(za - zp), print predicted, actual, and errors.

    print("Lab 01 scaffold loaded. Implement the tasks above in fk_predict() and run().")
