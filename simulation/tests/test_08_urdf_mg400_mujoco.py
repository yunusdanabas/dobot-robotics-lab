"""MG400 URDF pose hook in MuJoCo. Run as a script; unittest discover skips these."""

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
    [45, 20, 30, 25],
    [0, 60, 10, -45],
    [-30, 10, 40, 90],
]


def _fk_ref(q, L1=175.0, L2=175.0, Z_base=116.0):
    t2 = math.radians(q[1])
    t3 = math.radians(q[1] + q[2])
    reach = L1 * math.cos(t2) + L2 * math.cos(t3)
    z = Z_base + L1 * math.sin(t2) + L2 * math.sin(t3)
    x = reach * math.cos(math.radians(q[0]))
    y = reach * math.sin(math.radians(q[0]))
    return x, y, z, float(q[3])


def _angle_err(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def main():
    gui = os.environ.get("DOBOT_VIZ", "1") != "0"
    print(f"=== MG400 URDF in MuJoCo (GUI={'on' if gui else 'off'}) ===")

    from sim_mg400 import SimMG400

    try:
        sim = SimMG400(backend="mujoco", gui=gui)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  ERROR - MuJoCo backend unavailable: {exc}")
        return 1

    db, mv, feed = sim.connect()
    try:
        passes = 0
        for q in CONFIGS:
            j3_fw = q[1] + q[2]
            mv.JointMovJ(q[0], q[1], j3_fw, q[3])
            x, y, z, r = mv.get_sim_ee_pose()
            xr, yr, zr, rr = _fk_ref(q)
            pos_err = max(abs(x - xr), abs(y - yr), abs(z - zr))
            rot_err = _angle_err(r, rr)
            ok = pos_err < TOL_MM and rot_err < TOL_DEG
            tag = "PASS" if ok else f"FAIL dpos={pos_err:.3f}mm dr={rot_err:.3f}deg"
            print(f"  {tag}  q={q}  ->  ({x:7.2f}, {y:7.2f}, {z:7.2f}, {r:7.2f})")
            passes += int(ok)

        total = len(CONFIGS)
        print(f"  Result: {passes}/{total} passed")
        return 0 if passes == total else 1
    finally:
        sim.close()


if __name__ == "__main__":
    rc = main()
    # os._exit bypasses interpreter finalization that triggers MuJoCo/GLFW segfault on Linux
    os._exit(rc if rc is not None else 0)
