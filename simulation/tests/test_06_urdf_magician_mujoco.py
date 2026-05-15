"""Magician URDF pose hook in MuJoCo. Run as a script; unittest discover skips these."""

from __future__ import annotations

import math
import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))

TOL_MM = 1.0
TOL_DEG = 0.5

CONFIGS = [
    [0, 0, 0, 0],
    [0, 30, 20, 0],
    [45, 30, 15, 20],
    [0, 60, 0, -45],
    [-30, 25, 10, 90],
]


EXPECTED_SIM_POSES = {
    (0, 0, 0, 0): (147.00, 0.00, 135.00, 0.0),
    (0, 30, 20, 0): (205.63, 0.00, 66.64, 0.0),
    (45, 30, 15, 20): (148.13, 148.13, 78.87, 0.0),
    (0, 60, 0, -45): (263.91, 0.00, 67.50, 0.0),
    (-30, 25, 10, 90): (174.78, -100.91, 96.83, 0.0),
}


def _fk_ref(q):
    return EXPECTED_SIM_POSES[tuple(q)]


def _mode_home_regression(gui: bool) -> tuple[bool, str]:
    from sim_dobot import SimDobot
    expected = {
        "none": (147.0, 0.0, 135.0),
        "motor": (207.0, 0.0, 135.0),
        "suction": (207.0, 0.0, 65.0),
    }
    for mode, (ex, ey, ez) in expected.items():
        bot = SimDobot(backend="mujoco", gui=gui, tool=mode)
        try:
            bot.move_to(0.0, 0.0, 0.0, 0.0, wait=True, mode=_AngleMode())
            p = bot.get_pose().position
            if max(abs(p.x - ex), abs(p.y - ey), abs(p.z - ez)) > TOL_MM:
                return False, f"{mode} home mismatch got=({p.x:.2f},{p.y:.2f},{p.z:.2f}) exp=({ex:.2f},{ey:.2f},{ez:.2f})"
        finally:
            bot.close()
    return True, "mode home FK outputs match none/motor/suction expectations"


def _angle_err(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _pick_platform_axis(axes):
    world_z = (0.0, 0.0, 1.0)
    best_idx = max(range(3), key=lambda i: abs(_dot(axes[i], world_z)))
    sign = 1.0 if _dot(axes[best_idx], world_z) >= 0.0 else -1.0
    return best_idx, sign


def _orientation_regression(bot) -> tuple[bool, str]:
    # Sweep J2 and J3 independently at fixed J1/J4.
    configs = [
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 20.0, 0.0, 0.0],
        [0.0, 45.0, 0.0, 0.0],
        [0.0, 70.0, 0.0, 0.0],
        [0.0, 0.0, -30.0, 0.0],
        [0.0, 0.0, 20.0, 0.0],
        [0.0, 0.0, 50.0, 0.0],
    ]
    world_z = (0.0, 0.0, 1.0)
    parallel_cos_min = 0.995  # about 5.7 deg max tilt from vertical normal

    bot.move_to(*configs[0], wait=True, mode=_AngleMode())
    ref_axes = bot.get_visual_tool_axes()
    normal_idx, normal_sign = _pick_platform_axis(ref_axes)

    for q in configs:
        bot.move_to(*q, wait=True, mode=_AngleMode())
        axes = bot.get_visual_tool_axes()
        normal_dot = normal_sign * _dot(axes[normal_idx], world_z)
        if normal_dot < parallel_cos_min:
            return False, f"platform not parallel for q={q} (dot={normal_dot:.3f})"
    return True, "platform normal stayed parallel for J2/J3 sweeps"


class _AngleMode:
    name = "MOVJ_ANGLE"


def main():
    gui = os.environ.get("DOBOT_VIZ", "1") != "0"
    print(f"=== Magician URDF in MuJoCo (GUI={'on' if gui else 'off'}) ===")

    from sim_dobot import SimDobot

    try:
        bot = SimDobot(backend="mujoco", gui=gui)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  ERROR - MuJoCo backend unavailable: {exc}")
        return 1

    try:
        passes = 0
        for q in CONFIGS:
            bot.move_to(q[0], q[1], q[2], q[3], wait=True, mode=_AngleMode())
            x, y, z, r = bot.get_sim_ee_pose()
            xr, yr, zr, rr = _fk_ref(q)
            pos_err = max(abs(x - xr), abs(y - yr), abs(z - zr))
            rot_err = _angle_err(r, rr)
            ok = pos_err < TOL_MM and rot_err < TOL_DEG
            tag = "PASS" if ok else f"FAIL dpos={pos_err:.3f}mm dr={rot_err:.3f}deg"
            print(f"  {tag}  q={q}  ->  ({x:7.2f}, {y:7.2f}, {z:7.2f}, {r:7.2f})")
            passes += int(ok)

        total = len(CONFIGS)
        print(f"  Result: {passes}/{total} passed")
        mode_ok, mode_msg = _mode_home_regression(gui=False)
        print(f"  {'PASS' if mode_ok else 'FAIL'}  mode-home: {mode_msg}")
        orient_ok, orient_msg = _orientation_regression(bot)
        print(f"  {'PASS' if orient_ok else 'FAIL'}  orientation: {orient_msg}")
        return 0 if passes == total and orient_ok and mode_ok else 1
    finally:
        bot.close()


if __name__ == "__main__":
    rc = main()
    # os._exit bypasses interpreter finalization that triggers MuJoCo/GLFW segfault on Linux
    os._exit(rc if rc is not None else 0)
