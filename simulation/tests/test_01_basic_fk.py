"""SimDobot FK regression: _fk matches fk_magician_firmware for none/motor/suction tools. Run as a script; unittest discover skips these tests."""

import math
import sys
import os

# Make sim_dobot importable
_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))

from sim_dobot import _SimDobotBase
from kinematics import fk_magician_firmware

# ---------------------------------------------------------------------------
# Reference FK (firmware-direct kinematics - simulation source of truth)
# ---------------------------------------------------------------------------

def fk_reference(q, tool="none"):
    x, y, z, _ = fk_magician_firmware(
        q[0], q[1], q[2], q[3], tool=tool,
    )
    return x, y, z


# ---------------------------------------------------------------------------
# Test configurations
# ---------------------------------------------------------------------------

# (description, q=[q1,q2,q3,q4] in degrees)
TEST_CONFIGS = [
    ("home (0,0,0,0)",         [0,   0,   0,  0]),
    ("example pose (0,30,20,0)", [0,  30,  20,  0]),
    ("base-rotated (45,30,15,0)", [45, 30,  15,  0]),
    ("high reach (0,60,0,0)",   [0,  60,   0,  0]),
    ("elbow-up (0,45,30,0)",    [0,  45,  30,  0]),
    ("negative base (-30,25,10,0)", [-30, 25, 10, 0]),
]

TOLERANCE_MM = 0.5   # SimDobot FK must match reference within 0.5 mm
TOOLS = ("none", "motor", "suction")


# ---------------------------------------------------------------------------
# Test: FK formula consistency
# ---------------------------------------------------------------------------

def test_fk_consistency():
    """SimDobot._fk() must match fk_reference() within TOLERANCE_MM."""
    bot = _SimDobotBase.__new__(_SimDobotBase)
    _SimDobotBase.__init__(bot)

    print("=" * 65)
    print("Test 01 - FK formula consistency")
    print(f"  Tolerance: {TOLERANCE_MM} mm")
    print(f"  {'Config':<40} {'X':>7} {'Y':>7} {'Z':>7}  {'Status'}")
    print("-" * 65)

    passes = 0
    total = 0
    for tool in TOOLS:
        bot._tool = tool
        for desc, q in TEST_CONFIGS:
            x_ref, y_ref, z_ref = fk_reference(q, tool=tool)
            x_sim, y_sim, z_sim, _ = bot._fk(q)
            ex = abs(x_sim - x_ref)
            ey = abs(y_sim - y_ref)
            ez = abs(z_sim - z_ref)
            ok = ex < TOLERANCE_MM and ey < TOLERANCE_MM and ez < TOLERANCE_MM
            status = "PASS" if ok else f"FAIL (eX={ex:.2f} eY={ey:.2f} eZ={ez:.2f})"
            if ok:
                passes += 1
            total += 1
            print(f"  [{tool}] {desc:<31} {x_sim:>7.1f} {y_sim:>7.1f} {z_sim:>7.1f}  {status}")

    print("-" * 65)
    print(f"  Result: {passes}/{total} passed")
    return passes == total


# ---------------------------------------------------------------------------
# Test: get_pose() returns correct attribute structure
# ---------------------------------------------------------------------------

def test_pose_format():
    """get_pose() must return object with .position and .joints attributes."""
    bot = _SimDobotBase.__new__(_SimDobotBase)
    _SimDobotBase.__init__(bot)

    print("\nTest 01b - get_pose() attribute format")
    pose = bot.get_pose()

    checks = [
        ("pose.position.x", hasattr(pose, "position") and hasattr(pose.position, "x")),
        ("pose.position.y", hasattr(pose, "position") and hasattr(pose.position, "y")),
        ("pose.position.z", hasattr(pose, "position") and hasattr(pose.position, "z")),
        ("pose.position.r", hasattr(pose, "position") and hasattr(pose.position, "r")),
        ("pose.joints.j1",  hasattr(pose, "joints")   and hasattr(pose.joints,   "j1")),
        ("pose.joints.j2",  hasattr(pose, "joints")   and hasattr(pose.joints,   "j2")),
        ("pose.joints.j3",  hasattr(pose, "joints")   and hasattr(pose.joints,   "j3")),
        ("pose.joints.j4",  hasattr(pose, "joints")   and hasattr(pose.joints,   "j4")),
    ]

    passes = 0
    for name, ok in checks:
        status = "PASS" if ok else "FAIL"
        if ok:
            passes += 1
        print(f"  {status}  {name}")

    print(f"  Values: X={pose.position.x:.1f}  Y={pose.position.y:.1f}  "
          f"Z={pose.position.z:.1f}  J2={pose.joints.j2:.1f}")
    return passes == len(checks)


# ---------------------------------------------------------------------------
# Test: MOVJ_ANGLE call updates state correctly
# ---------------------------------------------------------------------------

def test_joint_move():
    """MOVJ_ANGLE-style calls must update state and FK consistently."""
    from sim_dobot import _is_joint_mode

    print("\nTest 01c - MOVJ_ANGLE state update (backend-free, motor mode)")

    class _MockAngleMode:
        name = "MOVJ_ANGLE"

        def __str__(self):
            return self.name

    class _MockSim(_SimDobotBase):
        def move_to(self, x=None, y=None, z=None, r=0, wait=True, mode=None):
            if not _is_joint_mode(mode):
                raise AssertionError("Expected MOVJ_ANGLE mode")
            q = [float(x), float(y), float(z), 0.0]
            self._q = q
            self._cartesian = self._fk(q)

    bot = _MockSim(tool="motor")

    q_test = [0.0, 30.0, 20.0, 0.0]
    bot.move_to(q_test[0], q_test[1], q_test[2], q_test[3],
                wait=True, mode=_MockAngleMode())

    x_actual, y_actual, z_actual, r_actual = bot._cartesian
    x_ref, y_ref, z_ref = fk_reference(q_test, tool="motor")

    ex = abs(x_actual - x_ref)
    ey = abs(y_actual - y_ref)
    ez = abs(z_actual - z_ref)
    ok = ex < TOLERANCE_MM and ey < TOLERANCE_MM and ez < TOLERANCE_MM
    status = "PASS" if ok else f"FAIL (eX={ex:.2f} eY={ey:.2f} eZ={ez:.2f})"
    print(f"  {status}  q={q_test}  ->  X={x_actual:.1f} Y={y_actual:.1f} Z={z_actual:.1f}")
    return ok


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    results = []
    results.append(test_fk_consistency())
    results.append(test_pose_format())
    results.append(test_joint_move())

    print("\n" + "=" * 65)
    passed = sum(results)
    total  = len(results)
    if passed == total:
        print(f"ALL TESTS PASSED ({passed}/{total})")
    else:
        print(f"SOME TESTS FAILED ({passed}/{total} passed)")
        sys.exit(1)
