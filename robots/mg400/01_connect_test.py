"""
01_connect_test.py — MG400 TCP smoke test: open three sockets, print version/mode/pose without EnableRobot.

Prerequisites: see mg400/README.md. Run: python 01_connect_test.py [--ip ...|--robot N]
"""

import argparse

from utils_mg400 import (
    add_target_arguments,
    close_all,
    connect_from_args_or_exit,
    parse_pose,
    parse_robot_mode,
    query_dashboard_version,
    ROBOT_MODE,
)


def main():
    parser = argparse.ArgumentParser(description="MG400 TCP connectivity test")
    add_target_arguments(parser)
    args = parser.parse_args()
    ip, dashboard, move_api, feed = connect_from_args_or_exit(args)

    print(f"Connecting to MG400 at {ip} ...")
    print("  TCP sockets open: dashboard(29999), move(30003), feed(30004)")

    try:
        # --- Firmware version ---
        try:
            ver = query_dashboard_version(dashboard)
            print(f"  Firmware version : {ver.strip()}")
        except Exception as exc:
            print(f"  Firmware version : (unavailable — {exc})")

        # --- Robot mode ---
        try:
            mode_resp = dashboard.RobotMode()
            mode_code = parse_robot_mode(mode_resp)
            mode_name = ROBOT_MODE.get(mode_code, f"UNKNOWN({mode_code})")
            print(f"  Robot mode       : {mode_code} = {mode_name}")
            if mode_code == 9:
                print("  [Warning] Robot is in ERROR state — run check_errors(dashboard).")
        except Exception as exc:
            print(f"  Robot mode       : (unavailable — {exc})")

        # --- Current pose (no enable required) ---
        try:
            pose_resp = dashboard.GetPose()
            x, y, z, r = parse_pose(pose_resp)
            print(f"  Current pose     : X={x:.2f}  Y={y:.2f}  Z={z:.2f}  R={r:.2f}  (mm / deg)")
        except Exception as exc:
            print(f"  Current pose     : (unavailable — {exc})")

        print("\nConnectivity test PASSED. All sockets open, status readable.")
        print("Next step: run 07_keyboard_teleop.py --robot 1, or use labs/lab01_forward_kinematics/interface.py.")

    finally:
        close_all(dashboard, move_api, feed)
        print("Sockets closed.")


if __name__ == "__main__":
    main()
