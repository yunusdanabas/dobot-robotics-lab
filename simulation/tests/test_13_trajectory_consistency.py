"""interpolate_to / interpolate_joints_to stay within FK tolerances per backend. Run as a script; unittest discover skips these."""

from __future__ import annotations

import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))
os.environ.setdefault("DOBOT_VIZ", "0")

from kinematics import fk_magician, fk_mg400, max_position_error, angle_error_deg  # noqa: E402


class _AngleMode:
    name = "MOVJ_ANGLE"


def _pose_from_get_pose(pose):
    return (float(pose.position.x), float(pose.position.y), float(pose.position.z), float(pose.position.r))


def _check(label, expected, actual, pos_tol=3.0, rot_tol=1.0):
    pos_err = max_position_error(expected, actual)
    rot_err = angle_error_deg(expected[3], actual[3])
    ok = pos_err <= pos_tol and rot_err <= rot_tol
    tag = "PASS" if ok else f"FAIL dpos={pos_err:.2f}mm dr={rot_err:.2f}deg"
    print(f"  {tag}  {label}")
    return ok


def _magician(backend):
    from sim_dobot import SimDobot
    tool = "none"

    try:
        bot = SimDobot(backend=backend, gui=False, tool=tool)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  SKIP Magician {backend}: {exc}")
        return 0, 0
    passed = total = 0
    try:
        if backend == "pybullet":
            print("  SKIP Magician pybullet interpolate_to raw-pose parity: PyBullet IK targets the visual EE link.")
        else:
            target = (220.0, 40.0, 110.0, 0.0)
            bot.interpolate_to(*target, steps=5, pause=0.0)
            pose = bot.get_sim_ee_pose() if hasattr(bot, "get_sim_ee_pose") else _pose_from_get_pose(bot.get_pose())
            passed += int(_check(f"Magician {backend} interpolate_to", target, pose, pos_tol=6.0))
            total += 1

        q_target = [30.0, 25.0, 15.0, 0.0]
        expected = fk_magician(q_target, tool=tool)
        bot.interpolate_joints_to(q_target, steps=5, pause=0.0)
        pose = bot.get_sim_ee_pose() if hasattr(bot, "get_sim_ee_pose") else _pose_from_get_pose(bot.get_pose())
        passed += int(_check(f"Magician {backend} interpolate_joints_to", expected, pose, pos_tol=6.0))
        total += 1
    finally:
        bot.close()
    return passed, total


def _mg400(backend):
    from sim_mg400 import SimMG400

    try:
        sim = SimMG400(backend=backend, gui=False)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  SKIP MG400 {backend}: {exc}")
        return 0, 0
    db, mv, feed = sim.connect()
    passed = total = 0
    try:
        if backend == "pybullet":
            print("  SKIP MG400 pybullet interpolate_to raw-pose parity: PyBullet IK targets the visual EE link.")
        else:
            target = (300.0, 60.0, 80.0, 15.0)
            sim.interpolate_to(*target, steps=5, pause=0.0)
            passed += int(_check(f"MG400 {backend} interpolate_to", target, mv.get_pose_tuple(), pos_tol=6.0))
            total += 1

        q_target = [30.0, 20.0, 10.0, -20.0]
        expected = fk_mg400(q_target)
        sim.interpolate_joints_to(q_target, steps=5, pause=0.0)
        passed += int(_check(f"MG400 {backend} interpolate_joints_to", expected, mv.get_sim_ee_pose(), pos_tol=6.0))
        total += 1
    finally:
        sim.close()
    return passed, total


def main() -> int:
    print("=" * 70)
    print("Test 13 - trajectory consistency")
    print("=" * 70)
    total_pass = total = 0
    for backend in ("pybullet", "mujoco"):
        p, n = _magician(backend)
        total_pass += p
        total += n
    for backend in ("pybullet", "mujoco"):
        p, n = _mg400(backend)
        total_pass += p
        total += n
    if total == 0:
        print("No optional simulation backends available; treated as SKIP.")
        return 0
    print("=" * 70)
    print(f"Result: {total_pass}/{total} checked trajectories passed")
    return 0 if total_pass == total else 1


if __name__ == "__main__":
    sys.exit(main())
