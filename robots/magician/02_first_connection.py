"""
02_first_connection.py — Connect with pydobotplus and print pose.

Run: python 02_first_connection.py
"""

import sys
from pydobotplus import Dobot

from utils import find_port, unpack_pose


def main():
    PORT = find_port()          # auto-detect; override with e.g. PORT = "COM3" or "/dev/ttyUSB0"

    if PORT is None:
        sys.exit("[Error] No serial port found. Run 01_find_port.py first.")

    print(f"Connecting on {PORT} ...\n")

    bot = Dobot(port=PORT)
    try:
        x, y, z, r, j1, j2, j3, j4 = unpack_pose(bot.get_pose())

        print("=== Current Pose (pydobotplus) ===")
        print(f"  Cartesian : X={x:.1f}  Y={y:.1f}  Z={z:.1f}  R={r:.1f}  mm/deg")
        print(f"  Joints    : J1={j1:.1f}  J2={j2:.1f}  J3={j3:.1f}  J4={j4:.1f}  deg")
    finally:
        bot.close()


if __name__ == "__main__":
    main()
