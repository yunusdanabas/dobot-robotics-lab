"""
Lab 02 - Jacobian / IK scaffold (Tasks 1-3).

Fill in jacobian_4d, run_task2_jacobian_test, and solve_ik_and_draw_square.
Run via interface.py.
"""

import numpy as np

import utils as U


def jacobian_4d(q, robot_type=None):
    """4x4 Jacobian: rows [dx,dy,dz,dpsi], cols [dq1..dq4]. See lab handout for structure."""
    _ = (q, robot_type)
    raise NotImplementedError("Task 1: implement jacobian_4d")


def run_task2_jacobian_test(robot, n_samples=20):
    """Task 2: finite-difference vs J @ dq for n_samples cases; print MSE per handout."""
    _ = (robot, n_samples)
    raise NotImplementedError("Task 2: implement Jacobian test and MSE")


def solve_ik_and_draw_square(robot):
    """Task 3: weighted Gauss-Newton IK and a square path in the plane."""
    _ = robot
    raise NotImplementedError("Task 3: implement weighted GN IK")


def run(robot):
    """Entry point called by interface.py after setup()."""
    print("Lab 2 scaffold started.")
    print("Implement Task 1/2/3 in this file, then run again.")
    _ = (np, U, robot)
