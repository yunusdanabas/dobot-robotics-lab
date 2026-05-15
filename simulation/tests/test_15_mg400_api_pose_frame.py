"""MG400 pedagogical fk_mg400 vs calibrated fk_mg400_api and sim GetPose strings. Run as a script; unittest discover skips these."""

from __future__ import annotations

import math
import os
import re
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))
os.environ.setdefault("DOBOT_VIZ", "0")

from kinematics import angle_error_deg, fk_mg400, fk_mg400_api, max_position_error  # noqa: E402

# fk_mg400() is the pedagogical student formula kept for simlab-chain
# cross-validation. fk_mg400_api() is the calibrated dashboard/GetPose frame.


POSE_RE = re.compile(r"\{([^}]*)\}")


def _parse_get_pose(response: str) -> tuple[float, float, float, float]:
    match = POSE_RE.search(response)
    if not match:
        raise ValueError(f"Unexpected GetPose response: {response!r}")
    values = [float(part.strip()) for part in match.group(1).split(",")]
    return values[0], values[1], values[2], values[3]


def _check_pose(label, expected, actual, pos_tol=1.0, rot_tol=0.5):
    pos_err = max_position_error(expected, actual)
    rot_err = angle_error_deg(expected[3], actual[3])
    ok = pos_err <= pos_tol and rot_err <= rot_tol
    tag = "PASS" if ok else f"FAIL dpos={pos_err:.2f}mm dr={rot_err:.2f}deg"
    print(f"  {tag}  {label:<28} expected={expected} actual=({actual[0]:.1f},{actual[1]:.1f},{actual[2]:.1f},{actual[3]:.1f})")
    return ok


def _mg400_backend(backend: str) -> tuple[int, int]:
    from sim_mg400 import SimMG400

    try:
        sim = SimMG400(backend=backend, gui=False)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  SKIP MG400 {backend}: {exc}")
        return 0, 0

    dashboard, move_api, _feed = sim.connect()
    checks = [
        [45.0, 0.0, 0.0, 0.0],
        [45.0, 20.0, 10.0, 5.0],
    ]
    passed = 0
    total = 0
    try:
        for q_body in checks:
            # JointMovJ firmware J3 = J2 + J3_body.
            move_api.JointMovJ(q_body[0], q_body[1], q_body[1] + q_body[2], q_body[3])
            # api_pose is from GetPose() which uses the calibrated FK (J2 from vertical, R=J1+J4).
            api_pose = _parse_get_pose(dashboard.GetPose())
            expected_api = fk_mg400_api(q_body)
            label = f"MG400 {backend} q={q_body}"
            passed += int(_check_pose(label, expected_api, api_pose))
            total += 1
            # simlab_pose is from the URDF pedagogical chain implementing the student formula.
            simlab_pose = move_api.get_sim_ee_pose()
            expected_sim = fk_mg400(q_body)   # old student formula: R=J4, J2 from horizontal
            passed += int(_check_pose(f"{label} simlab", expected_sim, simlab_pose))
            total += 1
            if q_body == [45.0, 0.0, 0.0, 0.0]:
                # Symmetry check uses simlab (old-formula geometry): X=Y=350/√2, Z=116.
                symmetric = abs(simlab_pose[0] - simlab_pose[1]) <= 1.0
                x_expected = 350.0 / math.sqrt(2.0)
                z_ok = abs(simlab_pose[2] - 116.0) <= 1.0
                x_ok = abs(simlab_pose[0] - x_expected) <= 1.0 and abs(simlab_pose[1] - x_expected) <= 1.0
                ok = symmetric and x_ok and z_ok
                tag = "PASS" if ok else "FAIL"
                print(f"  {tag}  MG400 {backend} simlab symmetry X={simlab_pose[0]:.1f} Y={simlab_pose[1]:.1f} Z={simlab_pose[2]:.1f}")
                passed += int(ok)
                total += 1
    finally:
        sim.close()
    return passed, total


def main() -> int:
    print("=" * 70)
    print("Test 15 - MG400 API-frame pose regression")
    print("=" * 70)
    total_pass = 0
    total = 0
    for backend in ("pybullet", "mujoco"):
        p, n = _mg400_backend(backend)
        total_pass += p
        total += n
    if total == 0:
        print("No optional MG400 simulation backends available; treated as SKIP.")
        return 0
    print("=" * 70)
    print(f"Result: {total_pass}/{total} API-frame checks passed")
    return 0 if total_pass == total else 1


if __name__ == "__main__":
    sys.exit(main())
