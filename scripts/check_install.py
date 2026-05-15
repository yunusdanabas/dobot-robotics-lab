"""Quick install sanity check for ME403 lab files."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _check_module(name: str, required: bool = False) -> bool:
    ok = importlib.util.find_spec(name) is not None
    status = "OK" if ok else ("MISSING" if required else "optional missing")
    print(f"{name:<12} {status}")
    return ok or not required


def main() -> int:
    print(f"Python: {sys.version.split()[0]}")
    print(f"Root:   {ROOT}")
    ok = True
    ok &= _check_module("numpy", required=True)
    ok &= _check_module("scipy", required=True)
    ok &= _check_module("matplotlib", required=True)
    _check_module("mujoco")
    _check_module("pybullet")
    _check_module("pydobotplus")
    _check_module("serial")
    _check_module("PyQt5")

    for rel in ("simulation/runtime/sim_dobot.py", "simulation/runtime/sim_mg400.py"):
        path = ROOT / rel
        print(f"{rel:<42} {'OK' if path.exists() else 'MISSING'}")
        ok &= path.exists()

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
