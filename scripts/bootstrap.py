"""Install/check common ME403 lab profiles.

Prepared for ME403 Introduction to Robotics by Yunus Emre Danabas.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REQ = ROOT / "requirements"


def _run(cmd: list[str]) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True)


def _pip_install(requirements: Path, *, dry_run: bool) -> None:
    if not requirements.exists():
        raise FileNotFoundError(requirements)
    cmd = [sys.executable, "-m", "pip", "install", "-r", str(requirements)]
    if dry_run:
        print("DRY RUN:", " ".join(cmd))
        return
    _run(cmd)


def _fetch_assets(args: list[str], *, dry_run: bool) -> None:
    cmd = [sys.executable, str(ROOT / "scripts" / "fetch_assets.py"), *args]
    if dry_run:
        print("DRY RUN:", " ".join(cmd))
        return
    _run(cmd)


def main() -> int:
    parser = argparse.ArgumentParser(description="Set up ME403 lab dependencies and optional assets.")
    parser.add_argument("--simulation", action="store_true", help="Install MuJoCo/PyBullet dependencies and fetch URDF assets")
    parser.add_argument("--magician", action="store_true", help="Install Magician hardware dependencies")
    parser.add_argument("--mg400", action="store_true", help="Install MG400 GUI dependencies and fetch the MG400 SDK")
    parser.add_argument("--full", action="store_true", help="Set up simulation, Magician, and MG400 profiles")
    parser.add_argument("--no-assets", action="store_true", help="Install packages only; do not clone/download vendor assets")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without running them")
    args = parser.parse_args()

    if not any((args.simulation, args.magician, args.mg400, args.full)):
        parser.print_help()
        return 0

    _pip_install(REQ / "base.txt", dry_run=args.dry_run)

    if args.simulation or args.full:
        _pip_install(REQ / "simulation_mujoco.txt", dry_run=args.dry_run)
        _pip_install(REQ / "simulation_pybullet.txt", dry_run=args.dry_run)
        if not args.no_assets:
            _fetch_assets(["--magician-urdf", "--mg400-urdf"], dry_run=args.dry_run)

    if args.magician or args.full:
        _pip_install(REQ / "magician_hardware.txt", dry_run=args.dry_run)

    if args.mg400 or args.full:
        _pip_install(REQ / "mg400_hardware.txt", dry_run=args.dry_run)
        if not args.no_assets:
            _fetch_assets(["--mg400-sdk"], dry_run=args.dry_run)

    print("\nSetup step completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
