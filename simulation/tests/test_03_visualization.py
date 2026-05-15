"""SimDobot XY square trajectory. Skip GUI with DOBOT_VIZ=0 or --no-gui. Run as a script; unittest discover skips these."""

import os
import sys

os.environ.setdefault("DOBOT_SIMULATION", "1")
_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_SIM_ROOT, "runtime"))

from sim_dobot import SimDobot


def run():
    # Headless: skip blocking GUI windows (useful for tests / SSH sessions).
    no_gui = ("--no-gui" in sys.argv) or (os.environ.get("DOBOT_VIZ", "1") == "0")

    backend = os.environ.get("DOBOT_SIM_BACKEND", "mujoco")
    print(f"[test_03] Starting visualization test (backend: {backend}, gui: {not no_gui})")

    # PyBullet factory honours the same env/flag via DOBOT_SIM_GUI.
    bot = SimDobot(gui=not no_gui)

    # Square trajectory in XY plane at Z=100 mm.
    waypoints = [
        (200,    0, 100, 0, "start"),
        (230,   60, 100, 0, "NE corner"),
        (170,   60, 100, 0, "NW corner"),
        (170,  -60, 100, 0, "SW corner"),
        (230,  -60, 100, 0, "SE corner"),
        (200,    0, 100, 0, "return"),
    ]

    print("\n[test_03] Moving through square trajectory:")
    print(f"  {'Label':<12} {'X':>7} {'Y':>7} {'Z':>7}")
    print("-" * 42)

    for x, y, z, r, label in waypoints:
        bot.move_to(x, y, z, r)
        pose = bot.get_pose()
        print(f"  {label:<12} {pose.position.x:>7.1f} {pose.position.y:>7.1f} {pose.position.z:>7.1f}")

    print("\n[test_03] Trajectory complete.")

    if no_gui:
        print("[test_03] Headless mode - skipping GUI window.")
        bot.close()
        print("[test_03] Done. viz OK (headless)")
        return

    # Interactive visualization
    if backend == "pybullet":
        print("\n[test_03] PyBullet GUI is already open.")
        print("          The arm should be visible in the PyBullet window.")
        print("          Close the window or press Ctrl+C to exit.")
        try:
            import time
            while True:
                bot._p.stepSimulation(physicsClientId=bot._client)
                time.sleep(1 / 60)
        except KeyboardInterrupt:
            pass
    else:
        if hasattr(bot, "show"):
            print("\n[test_03] Opening simulator visualization...")
            print("          A window will open for the selected backend.")
            print("          Close the window or press Ctrl+C to exit.")
            bot.show()
        else:
            print("\n[test_03] show() not available on this backend.")

    bot.close()
    print("[test_03] Done.")


if __name__ == "__main__":
    run()
