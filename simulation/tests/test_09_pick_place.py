"""Suction/ToolDO and full pick_place paths on Magician/MG400 (PyBullet and MuJoCo headless cases). Run as a script; unittest discover skips these."""

from __future__ import annotations

import io
import os
import sys
import time

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))
os.environ.setdefault("DOBOT_VIZ", "0")

from sim_dobot import SimDobot
from sim_mg400 import SimMG400


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pass(label: str) -> None:
    print(f"  PASS  {label}")


def _fail(label: str, reason: str) -> None:
    print(f"  FAIL  {label}: {reason}")


def _check(condition: bool, label: str, reason: str = "") -> bool:
    if condition:
        _pass(label)
    else:
        _fail(label, reason)
    return condition


# ---------------------------------------------------------------------------
# Test 1 - suck() on PyBullet backend (sphere lifecycle)
# ---------------------------------------------------------------------------

def test_02_suck_pybullet():
    print("Test 01 - suck() PyBullet backend (sphere lifecycle)")
    bot = SimDobot(backend="pybullet", gui=False, tool="suction")
    passed = True
    try:
        # Move to a known Cartesian position first
        bot.move_to(220, -60, 30, 0)

        # Before suck: no payload
        passed &= _check(bot._payload_id is None,
                         "payload_id is None before suck")

        # suck ON: sphere should be created
        bot.suck(True)
        passed &= _check(bot._payload_id is not None,
                         "payload_id set after suck(True)")

        # Move while holding: sphere should track EE (no crash)
        bot.move_to(220, 60, 30, 0)
        passed &= _check(bot._payload_id is not None,
                         "payload_id still set after move while holding")

        # suck OFF: sphere removed
        bot.suck(False)
        passed &= _check(bot._payload_id is None,
                         "payload_id None after suck(False)")

        # Double OFF: idempotent
        bot.suck(False)
        passed &= _check(bot._payload_id is None,
                         "double suck(False) is idempotent")

        # suck ON again, then close - close should detach cleanly
        bot.suck(True)
        passed &= _check(bot._payload_id is not None,
                         "payload_id set before close")
    except Exception as exc:
        _fail("suck PyBullet", str(exc))
        return False
    finally:
        bot.close()
    passed &= _check(True, "close() with active payload does not crash")
    return passed


# ---------------------------------------------------------------------------
# Test 2 - suck() on MuJoCo backend (print-only fallback)
# ---------------------------------------------------------------------------

def test_03_suck_mujoco():
    print("Test 02 - suck() MuJoCo backend (print-only)")
    bot = SimDobot(backend="mujoco", gui=False, tool="suction")
    passed = True
    try:
        bot.move_to(220, -60, 30, 0)
        bot.suck(True)
        bot.move_to(220, 60, 30, 0)
        bot.suck(False)
        # MuJoCo backend has no _payload_id attribute (no sphere)
        has_payload = hasattr(bot, "_payload_id")
        passed &= _check(not has_payload or bot._payload_id is None,
                         "MuJoCo: no physics payload (print-only)")
    except Exception as exc:
        _fail("suck MuJoCo", str(exc))
        return False
    finally:
        bot.close()
    return passed


# ---------------------------------------------------------------------------
# Test 3 - ToolDO() base class (print + stub response)
# ---------------------------------------------------------------------------

def test_04_tooldo_base():
    print("Test 03 - ToolDO() base class (stub response)")
    from sim_mg400 import _SimMG400Base
    obj = _SimMG400Base()
    passed = True
    try:
        resp = obj.ToolDO(1, 1)
        passed &= _check("ToolDO" in resp, "ToolDO returns response string")
        resp2 = obj.ToolDO(2, 0)
        passed &= _check(resp2 is not None, "ToolDO(2,0) returns without crash")
    except Exception as exc:
        _fail("ToolDO base", str(exc))
        return False
    return passed


# ---------------------------------------------------------------------------
# Test 4 - ToolDO() PyBullet backend (sphere lifecycle)
# ---------------------------------------------------------------------------

def test_05_tooldo_pybullet():
    print("Test 04 - ToolDO() PyBullet backend (sphere lifecycle)")
    sim = SimMG400(backend="pybullet", gui=False)
    dashboard, move_api, _ = sim.connect()
    passed = True
    try:
        move_api.MovJ(480, -80, 20, 0)

        passed &= _check(dashboard._payload_id is None,
                         "payload_id None before ToolDO")

        dashboard.ToolDO(1, 1)
        passed &= _check(dashboard._payload_id is not None,
                         "payload_id set after ToolDO(1,1)")

        # Move while holding
        move_api.MovJ(480, 80, 20, 0)
        passed &= _check(dashboard._payload_id is not None,
                         "payload_id still set after move while holding")

        # Ignore non-suction DO indices
        pre_id = dashboard._payload_id
        dashboard.ToolDO(2, 0)
        passed &= _check(dashboard._payload_id == pre_id,
                         "ToolDO(2,0) does not remove suction payload")

        dashboard.ToolDO(1, 0)
        passed &= _check(dashboard._payload_id is None,
                         "payload_id None after ToolDO(1,0)")

        # Idempotent off
        dashboard.ToolDO(1, 0)
        passed &= _check(dashboard._payload_id is None,
                         "double ToolDO(1,0) is idempotent")
    except Exception as exc:
        _fail("ToolDO PyBullet", str(exc))
        return False
    finally:
        sim.close()
    return passed


# ---------------------------------------------------------------------------
# Test 5 - ToolDO() MuJoCo backend (print-only)
# ---------------------------------------------------------------------------

def test_06_tooldo_mujoco():
    print("Test 05 - ToolDO() MuJoCo backend (print-only)")
    sim = SimMG400(backend="mujoco", gui=False)
    dashboard, move_api, _ = sim.connect()
    passed = True
    try:
        move_api.MovJ(280, -80, 20, 0)
        dashboard.ToolDO(1, 1)
        move_api.MovJ(280, 80, 20, 0)
        dashboard.ToolDO(1, 0)
        has_payload = hasattr(dashboard, "_payload_id")
        passed &= _check(not has_payload or dashboard._payload_id is None,
                         "MuJoCo: no physics payload (print-only)")
    except Exception as exc:
        _fail("ToolDO MuJoCo", str(exc))
        return False
    finally:
        sim.close()
    return passed


# ---------------------------------------------------------------------------
# Test 6 - Full pick-and-place sequence: Magician PyBullet
# ---------------------------------------------------------------------------

def test_07_full_magician_pybullet():
    print("Test 06 - Full pick-and-place sequence (Magician, PyBullet)")
    _HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(_HERE, "..", "..", "robots", "magician"))
    from utils import safe_move, go_home, JUMP_HEIGHT

    PICK_X, PICK_Y, PICK_Z = 220, -60, 30
    PLACE_X, PLACE_Y, PLACE_Z = 220, 60, 30
    LIFT = JUMP_HEIGHT

    bot = SimDobot(backend="pybullet", gui=False, tool="suction")
    passed = True
    try:
        go_home(bot)
        safe_move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        safe_move(bot, PICK_X, PICK_Y, PICK_Z, 0)
        bot.suck(True)
        time.sleep(0.05)
        passed &= _check(bot._payload_id is not None, "payload attached at pick")
        safe_move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z, 0)
        bot.suck(False)
        time.sleep(0.05)
        passed &= _check(bot._payload_id is None, "payload removed at place")
        safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        go_home(bot)
        q = bot._q
        passed &= _check(all(abs(a) < 1 for a in q), "returned to home joints approx [0,0,0,0]")
    except Exception as exc:
        _fail("full Magician PyBullet", str(exc))
        return False
    finally:
        bot.suck(False)
        bot.close()
    return passed


# ---------------------------------------------------------------------------
# Test 7 - Full pick-and-place sequence: MG400 PyBullet
# ---------------------------------------------------------------------------

def test_08_full_mg400_pybullet():
    print("Test 07 - Full pick-and-place sequence (MG400, PyBullet)")
    from sim_mg400 import READY_POSE_MG400

    PICK_X, PICK_Y, PICK_Z = 280, -80, 20
    PLACE_X, PLACE_Y, PLACE_Z = 280, 80, 20
    LIFT = 30

    sim = SimMG400(backend="pybullet", gui=False)
    dashboard, move_api, _ = sim.connect()
    passed = True
    try:
        move_api.MovJ(*READY_POSE_MG400)
        move_api.MovJ(PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        move_api.MovL(PICK_X, PICK_Y, PICK_Z, 0)
        dashboard.ToolDO(1, 1)
        time.sleep(0.05)
        passed &= _check(dashboard._payload_id is not None, "payload attached at pick")
        move_api.MovL(PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        move_api.MovJ(PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        move_api.MovL(PLACE_X, PLACE_Y, PLACE_Z, 0)
        dashboard.ToolDO(1, 0)
        time.sleep(0.05)
        passed &= _check(dashboard._payload_id is None, "payload removed at place")
        move_api.MovL(PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        move_api.MovJ(*READY_POSE_MG400)
        x, y, z, _ = dashboard.get_pose_tuple()
        rx, ry, rz, _ = READY_POSE_MG400
        passed &= _check(abs(x - rx) < 1 and abs(y - ry) < 1 and abs(z - rz) < 1,
                         "returned to READY_POSE")
    except Exception as exc:
        _fail("full MG400 PyBullet", str(exc))
        return False
    finally:
        try:
            dashboard.ToolDO(1, 0)
        except Exception:
            pass
        sim.close()
    return passed


# ---------------------------------------------------------------------------
# Test 8 - Full pick-and-place sequence: Magician MuJoCo
# ---------------------------------------------------------------------------

def test_09_full_magician_mujoco():
    print("Test 08 - Full pick-and-place sequence (Magician, MuJoCo)")
    _HERE = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(_HERE, "..", "..", "robots", "magician"))
    from utils import safe_move, go_home, JUMP_HEIGHT

    PICK_X, PICK_Y, PICK_Z = 220, -60, 30
    PLACE_X, PLACE_Y, PLACE_Z = 220, 60, 30
    LIFT = JUMP_HEIGHT

    bot = SimDobot(backend="mujoco", gui=False, tool="suction")
    passed = True
    try:
        go_home(bot)
        safe_move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        safe_move(bot, PICK_X, PICK_Y, PICK_Z, 0)
        bot.suck(True)
        safe_move(bot, PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z, 0)
        bot.suck(False)
        safe_move(bot, PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        go_home(bot)
        q = bot._q
        passed &= _check(all(abs(a) < 1 for a in q), "returned to home joints approx [0,0,0,0]")
        passed &= _check(True, "MuJoCo full sequence completes without crash")
    except Exception as exc:
        _fail("full Magician MuJoCo", str(exc))
        return False
    finally:
        bot.suck(False)
        bot.close()
    return passed


# ---------------------------------------------------------------------------
# Test 9 - Full pick-and-place sequence: MG400 MuJoCo
# ---------------------------------------------------------------------------

def test_10_full_mg400_mujoco():
    print("Test 09 - Full pick-and-place sequence (MG400, MuJoCo)")
    from sim_mg400 import READY_POSE_MG400

    PICK_X, PICK_Y, PICK_Z = 280, -80, 20
    PLACE_X, PLACE_Y, PLACE_Z = 280, 80, 20
    LIFT = 30

    sim = SimMG400(backend="mujoco", gui=False)
    dashboard, move_api, _ = sim.connect()
    passed = True
    try:
        move_api.MovJ(*READY_POSE_MG400)
        move_api.MovJ(PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        move_api.MovL(PICK_X, PICK_Y, PICK_Z, 0)
        dashboard.ToolDO(1, 1)
        move_api.MovL(PICK_X, PICK_Y, PICK_Z + LIFT, 0)
        move_api.MovJ(PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        move_api.MovL(PLACE_X, PLACE_Y, PLACE_Z, 0)
        dashboard.ToolDO(1, 0)
        move_api.MovL(PLACE_X, PLACE_Y, PLACE_Z + LIFT, 0)
        move_api.MovJ(*READY_POSE_MG400)
        passed &= _check(True, "MuJoCo full sequence completes without crash")
    except Exception as exc:
        _fail("full MG400 MuJoCo", str(exc))
        return False
    finally:
        try:
            dashboard.ToolDO(1, 0)
        except Exception:
            pass
        sim.close()
    return passed


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

TESTS = [
    test_02_suck_pybullet,
    test_03_suck_mujoco,
    test_04_tooldo_base,
    test_05_tooldo_pybullet,
    test_06_tooldo_mujoco,
    test_07_full_magician_pybullet,
    test_08_full_mg400_pybullet,
    test_09_full_magician_mujoco,
    test_10_full_mg400_mujoco,
]

if __name__ == "__main__":
    print("=" * 60)
    print("Test 09 - suction simulation + pick-and-place")
    print("=" * 60)
    results = []
    for fn in TESTS:
        ok = fn()
        results.append(ok)
        print()
    n_pass = sum(results)
    n_total = len(results)
    print("=" * 60)
    print(f"Result: {n_pass}/{n_total} tests passed")
    sys.exit(0 if n_pass == n_total else 1)
