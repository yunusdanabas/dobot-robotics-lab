"""Check Dobot Magician USB serial discovery."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UTILS = ROOT / "robots" / "magician" / "utils.py"


def _load_utils():
    spec = importlib.util.spec_from_file_location("me403_magician_utils", UTILS)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description="Find/check the Dobot Magician serial port.")
    parser.add_argument("--connect", action="store_true", help="Try a real connection with pydobotplus")
    args = parser.parse_args()

    utils = _load_utils()
    port = utils.find_port()
    if not port:
        print("No serial port found. Check USB, power, permissions, and DobotStudio.")
        return 1
    print(f"Detected Magician port: {port}")

    if args.connect:
        print(
            "WARNING: Calibrate/home with original Dobot software "
            "(DobotStudio or DobotLab) before Python control."
        )
        print("WARNING: Ensure DobotStudio/DobotLab is disconnected/closed before continuing.")
        from pydobotplus import Dobot

        bot = Dobot(port=port)
        try:
            print("Connected. Pose:", bot.get_pose())
        finally:
            bot.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
