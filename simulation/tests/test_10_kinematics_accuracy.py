"""Planar Magician/MG400 FK/IK roundtrip accuracy. Run as a script; unittest discover skips these."""

from __future__ import annotations

import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))

from kinematics import (  # noqa: E402
    angle_error_deg,
    fk_magician,
    fk_mg400,
    ik_magician,
    ik_mg400,
    max_position_error,
)

POS_TOL_MM = 0.5
ANGLE_TOL_DEG = 0.5

MAGICIAN_CONFIGS = [
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 30.0, 20.0, 0.0],
    [45.0, 30.0, 15.0, 20.0],
    [-30.0, 25.0, 10.0, -45.0],
]

MG400_CONFIGS = [
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 30.0, 20.0, 0.0],
    [45.0, 20.0, 30.0, 25.0],
    [-30.0, 10.0, 40.0, -45.0],
]


def _roundtrip(label, fk, ik, configs):
    print(f"\n{label} analytical FK -> IK -> FK")
    passes = 0
    for q in configs:
        target = fk(q)
        q_back = ik(*target, q0=q)
        pose_back = fk(q_back)
        pos_err = max_position_error(target, pose_back)
        rot_err = angle_error_deg(target[3], pose_back[3])
        branch_ok = q[2] == 0.0 or q[2] * q_back[2] >= 0.0
        ok = pos_err <= POS_TOL_MM and rot_err <= ANGLE_TOL_DEG and branch_ok
        tag = "PASS" if ok else f"FAIL dpos={pos_err:.3f}mm dr={rot_err:.3f}deg q_back={q_back}"
        print(f"  {tag}  q={q}")
        passes += int(ok)
    return passes, len(configs)


def main() -> int:
    print("=" * 70)
    print("Test 10 - analytical kinematics accuracy")
    print(f"Tolerances: {POS_TOL_MM} mm, {ANGLE_TOL_DEG} deg")
    print("=" * 70)

    m_pass, m_total = _roundtrip("Magician", fk_magician, ik_magician, MAGICIAN_CONFIGS)
    g_pass, g_total = _roundtrip("MG400", fk_mg400, ik_mg400, MG400_CONFIGS)
    total_pass = m_pass + g_pass
    total = m_total + g_total
    print("=" * 70)
    print(f"Result: {total_pass}/{total} passed")
    return 0 if total_pass == total else 1


if __name__ == "__main__":
    sys.exit(main())
