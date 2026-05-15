"""CollisionGuardConfig filters and contact_summaries formatting. Run as a script; unittest discover skips these."""

from __future__ import annotations

import os
import sys

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))
os.environ.setdefault("DOBOT_VIZ", "0")

from collision import CollisionContact, CollisionGuardConfig, contact_summaries, should_ignore_contact  # noqa: E402


def test_filters() -> bool:
    print("\nFilter checks")
    config = CollisionGuardConfig(
        enabled=True,
        ignored_link_pairs=frozenset({tuple(sorted(("link_1", "link_2")))}),
    )
    checks = [
        ("parent pair", should_ignore_contact("link_2", "link_1", config)),
        ("simlab prefix", should_ignore_contact("simlab_magician_tip", "link_5", config)),
        ("real contact", not should_ignore_contact("link_2", "link_6", config)),
    ]
    passed = 0
    for name, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += int(ok)
    sample = [CollisionContact("test", "self", "robot", "robot", "link_2", "link_6", distance=-0.01)]
    summary_ok = "link_2" in contact_summaries(sample)
    print(f"  {'PASS' if summary_ok else 'FAIL'}  contact summary")
    return passed == len(checks) and summary_ok


def test_pybullet_primitive_contact() -> bool:
    print("\nPyBullet primitive contact")
    try:
        import pybullet as p
    except ImportError as exc:
        print(f"  SKIP PyBullet unavailable: {exc}")
        return True

    cid = p.connect(p.DIRECT)
    try:
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], physicsClientId=cid)
        a = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, basePosition=[0, 0, 0], physicsClientId=cid)
        b = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=shape, basePosition=[0.04, 0, 0], physicsClientId=cid)
        for _ in range(5):
            p.stepSimulation(physicsClientId=cid)
        contacts = p.getContactPoints(a, b, physicsClientId=cid)
        if not contacts:
            contacts = p.getClosestPoints(a, b, distance=0.01, physicsClientId=cid)
        ok = len(contacts) > 0
        print(f"  {'PASS' if ok else 'FAIL'}  overlapping boxes produced {len(contacts)} contact(s)")
        return ok
    finally:
        p.disconnect(cid)


def test_runtime_smoke() -> bool:
    print("\nRuntime collision-guard smoke")
    try:
        from sim_dobot import SimDobot
        bot = SimDobot(backend="pybullet", gui=False, collision_guard=True)
    except (ImportError, FileNotFoundError, RuntimeError) as exc:
        print(f"  SKIP runtime backend unavailable: {exc}")
        return True
    try:
        bot.move_to(0, 0, 0, 0, mode=type("_AngleMode", (), {"name": "MOVJ_ANGLE"})())
        contacts = bot.get_collision_contacts()
        print(f"  PASS  runtime guard initialized, contacts={len(contacts)}")
        return True
    finally:
        bot.close()


def test_collision_guard_reverts_magician_pybullet() -> bool:
    """Synthetic contact: guarded move rejects the step and restores prior joint state."""
    print("\nCollision guard revert integration (Magician PyBullet)")
    try:
        from collision import CollisionContact
        from sim_dobot import SimDobot
    except (ImportError, FileNotFoundError) as exc:
        print(f"  SKIP import or URDF: {exc}")
        return True

    AngleMode = type("_AngleMode", (), {"name": "MOVJ_ANGLE"})()
    fake = CollisionContact(
        backend="pybullet",
        kind="self",
        body_a="magician",
        body_b="magician",
        link_a="link_5",
        link_b="link_6",
        distance=-0.001,
    )
    bot = SimDobot(backend="pybullet", gui=False, collision_guard=True)
    triggered = [False]
    original = bot.get_collision_contacts

    def patched_gc():
        if triggered[0]:
            return [fake]
        return original()

    bot.get_collision_contacts = patched_gc
    try:
        bot.move_to(0.0, 15.0, -20.0, 0.0, mode=AngleMode)
        q0 = tuple(bot._q[:])
        triggered[0] = True
        bot.move_to(70.0, -40.0, 50.0, -30.0, mode=AngleMode)
        tol = 1e-6
        ok_joint = all(abs(float(a) - float(b)) < tol for a, b in zip(bot._q, q0))
        ok_stop = bot.is_collision_stopped()
        if ok_joint and ok_stop:
            print("  PASS  guard reverted joints and flagged collision stop")
            return True
        print(f"  FAIL  revert={ok_joint} collision_stopped={ok_stop}")
        return False
    finally:
        bot.close()


def main() -> int:
    print("=" * 70)
    print("Test 14 - collision guard")
    print("=" * 70)
    results = [
        test_filters(),
        test_pybullet_primitive_contact(),
        test_runtime_smoke(),
        test_collision_guard_reverts_magician_pybullet(),
    ]
    passed = sum(int(v) for v in results)
    print("=" * 70)
    print(f"Result: {passed}/{len(results)} sections passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
