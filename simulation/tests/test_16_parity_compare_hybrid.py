"""Loads tools/debugging/robot_parity_diagnostic and exercises hybrid compare policy. Run as a script; unittest discover skips these."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile


_PACK_ROOT = Path(__file__).resolve().parents[2]
_DIAG_PATH = _PACK_ROOT / "tools" / "debugging" / "robot_parity_diagnostic.py"


def _load_diag_module():
    module_name = "me403_test16_robot_parity_diagnostic"
    spec = importlib.util.spec_from_file_location(module_name, _DIAG_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load diagnostic module from {_DIAG_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with path.open("w", encoding="utf-8") as fh:
        for record in records:
            fh.write(json.dumps(record) + "\n")


def _case_result(robot: str, suite: str, case: str, pose: tuple[float, float, float, float], joints: tuple[float, float, float, float]) -> dict:
    return {
        "type": "case_result",
        "status": "ok",
        "robot": robot,
        "suite": suite,
        "case": case,
        "pose_after": list(pose),
        "joints_after": list(joints),
    }


def _check(name: str, ok: bool) -> int:
    print(f"  {'PASS' if ok else 'FAIL'} {name}")
    return int(ok)


def main() -> int:
    print("=" * 70)
    print("Test 16 - Hybrid Magician compare policy")
    print("=" * 70)
    os.environ.setdefault("DOBOT_VIZ", "0")
    diag = _load_diag_module()
    passed = 0
    total = 0

    # Helper math sanity: preferred anchor resolution and normalization.
    keys = [
        ("magician", "joint", "joint_zero"),
        ("magician", "cartesian", "cart_ready"),
    ]
    anchor = diag._resolve_magician_anchor_key(keys, "joint_zero")
    total += 1
    passed += _check("resolve anchor case", anchor == ("magician", "joint", "joint_zero"))

    diff = {
        "sim_pose": (120.0, 60.0, 30.0, 180.0),
        "real_pose": (100.0, 50.0, 20.0, 0.0),
    }
    metrics = diag._magician_hybrid_metrics(diff, (20.0, 10.0, 10.0))
    total += 1
    passed += _check("baseline normalized xyz is zero", abs(metrics["norm_max_position_error_mm"]) < 1e-9)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        sim_path = tmp / "sim.jsonl"
        real_path = tmp / "real.jsonl"

        sim_records = [
            _case_result("magician", "joint", "joint_zero", (120.0, 60.0, 30.0, 180.0), (0.0, 0.0, 0.0, 0.0)),
            _case_result("magician", "joint", "base_left_small", (130.0, 60.0, 30.0, 179.0), (10.0, 5.0, 4.0, 0.0)),
        ]
        real_records = [
            _case_result("magician", "joint", "joint_zero", (100.0, 50.0, 20.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
            _case_result("magician", "joint", "base_left_small", (110.0, 50.0, 20.0, 0.0), (10.2, 5.0, 4.0, 0.0)),
        ]
        _write_jsonl(sim_path, sim_records)
        _write_jsonl(real_path, real_records)

        args_default = argparse.Namespace(
            sim_jsonl=str(sim_path),
            real_jsonl=str(real_path),
            pos_tol=10.0,
            rot_tol=5.0,
            magician_hybrid=False,
            anchor_case="joint_zero",
            joint_tol=5.0,
        )
        default_exit = diag.cmd_compare(args_default)
        total += 1
        passed += _check("default compare fails large rotation", default_exit == 1)

        args_hybrid = argparse.Namespace(
            sim_jsonl=str(sim_path),
            real_jsonl=str(real_path),
            pos_tol=10.0,
            rot_tol=5.0,
            magician_hybrid=True,
            anchor_case="joint_zero",
            joint_tol=5.0,
        )
        hybrid_exit = diag.cmd_compare(args_hybrid)
        total += 1
        passed += _check("hybrid compare ignores R and passes", hybrid_exit == 0)

        # Force a joint mismatch on non-anchor case: should fail hybrid mode.
        real_records_bad_joint = [
            _case_result("magician", "joint", "joint_zero", (100.0, 50.0, 20.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
            _case_result("magician", "joint", "base_left_small", (110.0, 50.0, 20.0, 0.0), (20.0, 5.0, 4.0, 0.0)),
        ]
        _write_jsonl(real_path, real_records_bad_joint)
        hybrid_bad_joint_exit = diag.cmd_compare(args_hybrid)
        total += 1
        passed += _check("hybrid compare fails joint tolerance", hybrid_bad_joint_exit == 1)

    print("=" * 70)
    print(f"Result: {passed}/{total} checks passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
