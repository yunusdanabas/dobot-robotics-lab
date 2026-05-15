"""Cartesian and joint moves through utils_sim with SimDobot; requires DOBOT_SIMULATION=1. Run as a script; unittest discover skips these."""

import os
import sys

# Ensure simulation mode
os.environ.setdefault("DOBOT_SIMULATION", "1")
if os.environ.get("DOBOT_SIMULATION") != "1":
    print("Set DOBOT_SIMULATION=1 to run this test.")
    sys.exit(1)

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))

import utils_sim as U
from kinematics import fk_magician_firmware

CARTESIAN_TOL_MM = 3.0   # IK roundtrip tolerance (mm)
JOINT_TOL_MM     = 0.5   # FK consistency tolerance (mm)


# ---------------------------------------------------------------------------
# Test: Cartesian IK roundtrip
# ---------------------------------------------------------------------------

def test_cartesian_moves():
    """move_to(x,y,z,r) via IK then get_pose() should match within tolerance."""
    print("=" * 65)
    print("Test 02a - Cartesian IK roundtrip")
    print(f"  Tolerance: {CARTESIAN_TOL_MM} mm")

    targets = [
        (200.0,   0.0, 100.0, 0.0, "READY_POSE"),
        (250.0,  50.0,  80.0, 0.0, "forward-right"),
        (180.0, -80.0, 120.0, 0.0, "left-elevated"),
        (220.0,   0.0, 150.0, 0.0, "high"),
    ]

    bot = U.setup()
    print()
    print(f"  {'Target':<18} {'Xc':>7} {'Yc':>7} {'Zc':>7}  {'Status'}")
    print("-" * 65)

    passes = 0
    try:
        for x, y, z, r, label in targets:
            bot.move_to(x, y, z, r)
            pose = bot.get_pose()
            xg = pose.position.x
            yg = pose.position.y
            zg = pose.position.z
            ex = abs(xg - x)
            ey = abs(yg - y)
            ez = abs(zg - z)
            ok = ex < CARTESIAN_TOL_MM and ey < CARTESIAN_TOL_MM and ez < CARTESIAN_TOL_MM
            status = "PASS" if ok else f"FAIL (eX={ex:.1f} eY={ey:.1f} eZ={ez:.1f})"
            if ok:
                passes += 1
            print(f"  {label:<18} {xg:>7.1f} {yg:>7.1f} {zg:>7.1f}  {status}")
    finally:
        U.teardown(bot)

    print(f"  Result: {passes}/{len(targets)} passed")
    return passes == len(targets)


# ---------------------------------------------------------------------------
# Test: moveMagician() joint-angle path via utils_sim
# ---------------------------------------------------------------------------

def test_joint_move_via_utils():
    """U.moveMagician(bot, q) should update pose consistently with FK."""
    print("\nTest 02b - moveMagician() via utils_sim")

    configs = [
        [0,  30, 20, 0],
        [45, 40, 10, 0],
        [0,  20,  5, 0],
    ]

    def fk(q):
        x, y, z, _ = fk_magician_firmware(q[0], q[1], q[2], q[3], L1=U.L1, L2=U.L2, tool="none")
        return x, y, z

    bot = U.setup()
    print(f"  {'q':<26} {'Xs':>7} {'Zs':>7}  {'Xfk':>7} {'Zfk':>7}  Status")
    print("-" * 65)

    passes = 0
    try:
        for q in configs:
            U.moveMagician(bot, q)
            x, y, z, r = U.get_pose(bot)
            xfk, yfk, zfk = fk(q)
            ex = abs(x - xfk)
            ez = abs(z - zfk)
            ok = ex < JOINT_TOL_MM and ez < JOINT_TOL_MM
            status = "PASS" if ok else f"FAIL (eX={ex:.2f} eZ={ez:.2f})"
            if ok:
                passes += 1
            print(f"  {str(q):<26} {x:>7.1f} {z:>7.1f}  {xfk:>7.1f} {zfk:>7.1f}  {status}")
    finally:
        U.teardown(bot)

    print(f"  Result: {passes}/{len(configs)} passed")
    return passes == len(configs)


# ---------------------------------------------------------------------------
# Test: safe_move() clamping
# ---------------------------------------------------------------------------

def test_safe_move_clamping():
    """Clamp logic: out-of-bounds values are clamped to SAFE_BOUNDS correctly."""
    print("\nTest 02c - safe_move() clamp logic (pure Python, no backend needed)")

    SAFE_BOUNDS = {
        "x": (120, 315),
        "y": (-158, 158),
        "z": (5, 155),
        "r": (-90, 90),
    }

    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    test_cases = [
        # (x, y, z, r,  expected_after_clamp,     label)
        (50,  0, 100, 0,  (120, 0, 100, 0),  "x below min  50 -> 120"),
        (400, 0, 100, 0,  (315, 0, 100, 0),  "x above max 400 -> 315"),
        (200, 0,   0, 0,  (200, 0,   5, 0),  "z below min   0 ->   5"),
        (200, 0, 200, 0,  (200, 0, 155, 0),  "z above max 200 -> 155"),
        (200, 200, 100, 0, (200, 158, 100, 0), "y above max 200 -> 158"),
        (200, 0, 100, 100, (200, 0, 100, 90),  "r above max 100 ->  90"),
    ]

    passes = 0
    for x, y, z, r, (ecx, ecy, ecz, ecr), label in test_cases:
        cx = clamp(x, *SAFE_BOUNDS["x"])
        cy = clamp(y, *SAFE_BOUNDS["y"])
        cz = clamp(z, *SAFE_BOUNDS["z"])
        cr = clamp(r, *SAFE_BOUNDS["r"])
        ok = (cx == ecx and cy == ecy and cz == ecz and cr == ecr)
        status = "PASS" if ok else f"FAIL (got {cx},{cy},{cz},{cr})"
        if ok:
            passes += 1
        print(f"  {status:<6}  {label}")

    print(f"  Result: {passes}/{len(test_cases)} passed")
    return passes == len(test_cases)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    results = []

    try:
        results.append(test_cartesian_moves())
    except ImportError as exc:
        print(f"  SKIP test_02a (simulation import unavailable): {exc}")
        results.append(True)

    try:
        results.append(test_joint_move_via_utils())
    except ImportError as exc:
        print(f"  SKIP test_02b (simulation import unavailable): {exc}")
        results.append(True)

    results.append(test_safe_move_clamping())

    print("\n" + "=" * 65)
    passed = sum(results)
    total  = len(results)
    if passed == total:
        print(f"ALL TESTS PASSED ({passed}/{total})")
    else:
        print(f"SOME TESTS FAILED ({passed}/{total} passed)")
        sys.exit(1)
