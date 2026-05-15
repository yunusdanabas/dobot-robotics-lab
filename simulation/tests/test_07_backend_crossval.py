"""Cross-check PyBullet/MuJoCo raw poses (Magician + MG400). Run as a script; unittest discover skips these."""

from __future__ import annotations

import itertools
import math
import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))
os.environ.setdefault("DOBOT_VIZ", "0")

TOL_MM = 2.0
TOL_DEG = 0.5


def _build_magician_configs():
    base = [
        [q1, q2, q3, 0.0]
        for q1 in (-45.0, 0.0, 45.0)
        for q2 in (0.0, 20.0, 40.0)
        for q3 in (0.0, 10.0, 20.0)
    ]
    return base + [
        [0.0, 20.0, 10.0, -90.0],
        [0.0, 20.0, 10.0, 45.0],
        [45.0, 40.0, 20.0, 90.0],
    ]


def _build_mg400_configs():
    base = [
        [q1, q2, q3, 0.0]
        for q1 in (-60.0, 0.0, 60.0)
        for q2 in (0.0, 20.0, 40.0)
        for q3 in (0.0, 10.0, 20.0)
    ]
    return base + [
        [0.0, 20.0, 10.0, -90.0],
        [0.0, 20.0, 10.0, 45.0],
        [60.0, 40.0, 20.0, 90.0],
    ]


MAGICIAN_CONFIGS = _build_magician_configs()
MG400_CONFIGS = _build_mg400_configs()


def _fk_mg400(q, L1=175.0, L2=175.0, Z_base=116.0):
    t2 = math.radians(q[1])
    t3 = math.radians(q[1] + q[2])
    reach = L1 * math.cos(t2) + L2 * math.cos(t3)
    z = Z_base + L1 * math.sin(t2) + L2 * math.sin(t3)
    x = reach * math.cos(math.radians(q[0]))
    y = reach * math.sin(math.radians(q[0]))
    return (x, y, z, float(q[3]))


def _position_err(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def _angle_err(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _pairwise_err(p1, p2):
    return _position_err(p1, p2), _angle_err(p1[3], p2[3])


class _AngleMode:
    name = "MOVJ_ANGLE"


def _pose_from_get_pose(pose):
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.position.r),
    )


def cross_magician():
    from sim_dobot import SimDobot

    backends = {}
    for backend in ("pybullet", "mujoco"):
        try:
            backends[backend] = SimDobot(backend=backend, gui=False)
        except (ImportError, FileNotFoundError, RuntimeError) as exc:
            print(f"  ERROR - cannot initialize Magician backend '{backend}': {exc}")
            return 0, 0, False

    max_pos_err = 0.0
    max_rot_err = 0.0
    passed = 0
    try:
        for q in MAGICIAN_CONFIGS:
            poses = {}
            for name, bot in backends.items():
                bot.move_to(q[0], q[1], q[2], q[3], wait=True, mode=_AngleMode())
                poses[name] = bot.get_sim_ee_pose()
            ok = True
            for p1, p2 in itertools.combinations(poses.values(), 2):
                pos_err, rot_err = _pairwise_err(p1, p2)
                max_pos_err = max(max_pos_err, pos_err)
                max_rot_err = max(max_rot_err, rot_err)
                if pos_err > TOL_MM or rot_err > TOL_DEG:
                    ok = False
            if not ok:
                print(f"  FAIL Magician q={q} poses={poses}")
            passed += int(ok)
    finally:
        for bot in backends.values():
            bot.close()

    total = len(MAGICIAN_CONFIGS)
    print(f"  Magician: {passed}/{total} within {TOL_MM} mm and {TOL_DEG} deg")
    print(f"    max pairwise error: {max_pos_err:.3f} mm, {max_rot_err:.3f} deg")
    return passed, total, True


def cross_mg400():
    from sim_mg400 import SimMG400

    backends = {}
    for backend in ("pybullet", "mujoco"):
        try:
            sim = SimMG400(backend=backend, gui=False)
        except (ImportError, FileNotFoundError, RuntimeError) as exc:
            print(f"  ERROR - cannot initialize MG400 backend '{backend}': {exc}")
            return 0, 0, False
        db, mv, feed = sim.connect()
        backends[backend] = (sim, db, mv)

    max_pos_err = 0.0
    max_rot_err = 0.0
    passed = 0
    try:
        for q in MG400_CONFIGS:
            ref = _fk_mg400(q)
            poses = {"fk": ref}
            for name, (_, _, mv) in backends.items():
                j3_fw = q[1] + q[2]
                mv.JointMovJ(q[0], q[1], j3_fw, q[3])
                poses[name] = mv.get_sim_ee_pose()
            ok = True
            for p1, p2 in itertools.combinations(poses.values(), 2):
                pos_err, rot_err = _pairwise_err(p1, p2)
                max_pos_err = max(max_pos_err, pos_err)
                max_rot_err = max(max_rot_err, rot_err)
                if pos_err > TOL_MM or rot_err > TOL_DEG:
                    ok = False
            if not ok:
                print(f"  FAIL MG400 q={q} poses={poses}")
            passed += int(ok)
    finally:
        for sim, _, _ in backends.values():
            sim.close()

    total = len(MG400_CONFIGS)
    print(f"  MG400: {passed}/{total} within {TOL_MM} mm and {TOL_DEG} deg")
    print(f"    max pairwise error: {max_pos_err:.3f} mm, {max_rot_err:.3f} deg")
    return passed, total, True


def main():
    print("=" * 68)
    print("Test 07 - cross-backend FK parity")
    print(f"  Magician configs: {len(MAGICIAN_CONFIGS)}")
    print(f"  MG400 configs   : {len(MG400_CONFIGS)}")
    print(f"  Tolerances      : {TOL_MM} mm, {TOL_DEG} deg")
    print("=" * 68)

    m_pass, m_total, m_ok = cross_magician()
    g_pass, g_total, g_ok = cross_mg400()
    if not (m_ok and g_ok):
        return 1

    total_pass = m_pass + g_pass
    total_all = m_total + g_total
    print("=" * 68)
    print(f"Result: {total_pass}/{total_all} configurations within tolerance")
    return 0 if total_pass == total_all else 1


if __name__ == "__main__":
    sys.exit(main())
