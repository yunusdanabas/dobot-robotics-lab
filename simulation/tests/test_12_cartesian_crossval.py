"""Cartesian move_to agreement across SimDobot/SimMG400 backends. Run as a script; unittest discover skips these."""

from __future__ import annotations

import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))
os.environ.setdefault("DOBOT_VIZ", "0")

from kinematics import angle_error_deg, max_position_error  # noqa: E402

MAGICIAN_TARGETS = [
    (200.0, 0.0, 100.0, 0.0),
    (230.0, 40.0, 110.0, 0.0),
    (180.0, -70.0, 120.0, 0.0),
]
MG400_TARGETS = [
    (300.0, 0.0, 50.0, 0.0),
    (320.0, 60.0, 80.0, 30.0),
    (260.0, -80.0, 90.0, -45.0),
]


def _pose_from_get_pose(pose):
    return (float(pose.position.x), float(pose.position.y), float(pose.position.z), float(pose.position.r))


def _check_pose(label, target, pose, pos_tol, rot_tol):
    pos_err = max_position_error(target, pose)
    rot_err = angle_error_deg(target[3], pose[3])
    ok = pos_err <= pos_tol and rot_err <= rot_tol
    tag = "PASS" if ok else f"FAIL dpos={pos_err:.2f}mm dr={rot_err:.2f}deg"
    print(f"  {tag}  {label:<18} target={target} pose=({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f},{pose[3]:.1f})")
    return ok


def _magician_backend(backend):
    from sim_dobot import SimDobot

    if backend == "pybullet":
        print("  SKIP Magician pybullet Cartesian raw-pose parity: PyBullet IK targets the visual EE link.")
        return 0, 0

    try:
        bot = SimDobot(backend=backend, gui=False)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  SKIP Magician {backend}: {exc}")
        return 0, 0
    pos_tol = 5.0 if backend == "pybullet" else 2.0
    rot_tol = 1.0
    passed = 0
    try:
        for target in MAGICIAN_TARGETS:
            bot.move_to(*target)
            pose = bot.get_sim_ee_pose() if hasattr(bot, "get_sim_ee_pose") else _pose_from_get_pose(bot.get_pose())
            passed += int(_check_pose(f"Magician {backend}", target, pose, pos_tol, rot_tol))
    finally:
        bot.close()
    return passed, len(MAGICIAN_TARGETS)


def _mg400_backend(backend):
    from sim_mg400 import SimMG400

    try:
        sim = SimMG400(backend=backend, gui=False)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  SKIP MG400 {backend}: {exc}")
        return 0, 0
    db, mv, feed = sim.connect()
    passed = 0
    try:
        for target in MG400_TARGETS:
            mv.MovJ(*target)
            pose = mv.get_pose_tuple()
            passed += int(_check_pose(f"MG400 {backend}", target, pose, 3.0, 1.0))
    finally:
        sim.close()
    return passed, len(MG400_TARGETS)


def main() -> int:
    print("=" * 70)
    print("Test 12 - Cartesian backend cross-validation")
    print("=" * 70)
    total_pass = 0
    total = 0
    for backend in ("pybullet", "mujoco"):
        p, n = _magician_backend(backend)
        total_pass += p
        total += n
    for backend in ("pybullet", "mujoco"):
        p, n = _mg400_backend(backend)
        total_pass += p
        total += n
    if total == 0:
        print("No optional simulation backends available; treated as SKIP.")
        return 0
    print("=" * 70)
    print(f"Result: {total_pass}/{total} checked poses passed")
    return 0 if total_pass == total else 1


if __name__ == "__main__":
    sys.exit(main())
