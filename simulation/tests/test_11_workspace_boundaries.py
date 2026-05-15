"""Planar 2R reach/singularity checks (MAGICIAN/MG400 legacy models). Run as a script; unittest discover skips these."""

from __future__ import annotations

import math
import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))

from kinematics import MAGICIAN, MG400, fk_2r_planar, ik_2r_planar, max_position_error  # noqa: E402

SINGULAR_TOL_MM = 0.5


def _finite(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def _check_model(label, model):
    print(f"\n{label} workspace boundaries")
    checks = []

    full_extension = (model.l1 + model.l2, 0.0, model.z_base, 0.0)
    q_full = ik_2r_planar(*full_extension, model, q0=[0.0, 0.0, 0.0, 0.0])
    pose_full = fk_2r_planar(q_full, model)
    checks.append(("full extension", max_position_error(full_extension, pose_full) <= SINGULAR_TOL_MM and abs(q_full[2]) <= 1.0))

    out_of_reach = (model.l1 + model.l2 + 50.0, 0.0, model.z_base, 0.0)
    q_oor = ik_2r_planar(*out_of_reach, model, q0=[0.0, 0.0, 0.0, 0.0])
    pose_oor = fk_2r_planar(q_oor, model)
    checks.append(("out-of-reach finite", _finite(q_oor + list(pose_oor))))
    checks.append(("out-of-reach clamps to max reach", abs(pose_oor[0] - (model.l1 + model.l2)) <= SINGULAR_TOL_MM))

    high = (0.0, 0.0, model.z_base + model.l1 + model.l2, 0.0)
    q_high = ik_2r_planar(*high, model, q0=[0.0, 80.0, 0.0, 0.0])
    pose_high = fk_2r_planar(q_high, model)
    checks.append(("vertical reach finite", _finite(q_high + list(pose_high))))

    base_rot = [90.0, 20.0, 10.0, 0.0]
    x, y, z, r = fk_2r_planar(base_rot, model)
    checks.append(("base rotation X near zero", abs(x) <= 1e-9 and y > 0.0))

    passed = 0
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += int(ok)
    return passed, len(checks)


def main() -> int:
    print("=" * 70)
    print("Test 11 - workspace boundaries")
    print("=" * 70)
    m_pass, m_total = _check_model("Magician", MAGICIAN)
    g_pass, g_total = _check_model("MG400", MG400)
    total_pass = m_pass + g_pass
    total = m_total + g_total
    print("=" * 70)
    print(f"Result: {total_pass}/{total} passed")
    return 0 if total_pass == total else 1


if __name__ == "__main__":
    sys.exit(main())
