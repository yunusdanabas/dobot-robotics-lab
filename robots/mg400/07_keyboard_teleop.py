"""
07_keyboard_teleop.py — MG400 teleop: Tk GUI by default; --mode keyboard for terminal keys.

Run: python 07_keyboard_teleop.py [--mode gui|keyboard] [--robot N|--ip ADDR] ...
"""

import argparse
import sys
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk

def _discover_root(start: Path) -> Path:
    for candidate in [start, *start.parents]:
        if (candidate / "terminal_keys.py").exists():
            return candidate
    return start


_ROOT = _discover_root(Path(__file__).resolve().parent)
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from terminal_keys import TerminalKeyReader
from gui_controls import bind_hold_button, labeled_value, schedule_periodic

from utils_mg400 import (
    add_target_arguments,
    close_all,
    check_errors,
    connect_from_args_or_exit,
    go_home,
    parse_pose,
)
from viz_mg400 import RobotViz

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SUCTION_DO = 1    # tool digital output index for suction pump
JOG_SPEED  = 20   # % — teleop jog speed (lower = more precise)

# Minimum interval between MoveJog("") + pose reads (s)
STATUS_INTERVAL = 0.2


# Map key → MoveJog axis string
_KEY_TO_JOG = {
    "right": "X+", "d": "X+",
    "left":  "X-", "a": "X-",
    "up":    "Y+", "w": "Y+",
    "down":  "Y-", "s": "Y-",
    "r":     "Z+",
    "f":     "Z-",
    "q":     "Rx+",  # end-effector +R (mapped as Rx for 4-axis)
    "e":     "Rx-",
}


def _run_keyboard(move_api, dashboard):
    if not TerminalKeyReader.require_tty():
        sys.exit("[Error] Keyboard mode requires an interactive terminal (TTY).")
    suction_on = False
    current_jog = None
    last_status = 0.0
    with TerminalKeyReader() as keys:
        while True:
            now = time.perf_counter()
            key = keys.read_key(timeout_s=0.02)
            if key is not None:
                if key == "esc":
                    break
                if key == " ":
                    if current_jog:
                        move_api.MoveJog("")
                        current_jog = None
                    suction_on = not suction_on
                    dashboard.ToolDO(SUCTION_DO, 1 if suction_on else 0)
                elif key == "h":
                    if current_jog:
                        move_api.MoveJog("")
                        current_jog = None
                    go_home(move_api)
                elif key in _KEY_TO_JOG:
                    axis = _KEY_TO_JOG[key]
                    if axis != current_jog:
                        if current_jog:
                            move_api.MoveJog("")
                        move_api.MoveJog(axis)
                        current_jog = axis
            else:
                if current_jog:
                    move_api.MoveJog("")
                    current_jog = None

            if (now - last_status) >= STATUS_INTERVAL:
                try:
                    x, y, z, r = parse_pose(dashboard.GetPose())
                    print(
                        f"  X={x:7.1f}  Y={y:7.1f}  Z={z:7.1f}  R={r:6.1f}"
                        f"  Jog={current_jog or '---':<4}  Suction={'ON' if suction_on else 'OFF'}",
                        end="\r",
                    )
                except Exception:
                    pass
                last_status = now
    if current_jog:
        move_api.MoveJog("")
    if suction_on:
        dashboard.ToolDO(SUCTION_DO, 0)
    print()


def _set_jog(move_api, state, axis):
    if axis == state["jog"]:
        return
    if state["jog"] is not None:
        move_api.MoveJog("")
    if axis is not None:
        move_api.MoveJog(axis)
    state["jog"] = axis


def _run_gui(move_api, dashboard):
    root = tk.Tk()
    root.title("MG400 Teleop")
    root.geometry("540x430")
    state = {"jog": None, "suction": False}
    speed = tk.IntVar(value=JOG_SPEED)

    outer = ttk.Frame(root, padding=12)
    outer.pack(fill="both", expand=True)
    ttk.Label(outer, text="MG400 GUI Teleop", font=("TkDefaultFont", 12, "bold")).pack(anchor="w")

    pose_labels = {}
    for axis in ("X", "Y", "Z", "R"):
        row, val = labeled_value(outer, f"{axis}:")
        row.pack(anchor="w")
        pose_labels[axis] = val

    jog = ttk.LabelFrame(outer, text="Jog (hold button)")
    jog.pack(fill="x", pady=10)
    mapping = [("X-", "X-"), ("X+", "X+"), ("Y-", "Y-"), ("Y+", "Y+"), ("Z-", "Z-"), ("Z+", "Z+"), ("R-", "Rx-"), ("R+", "Rx+")]
    for idx, (label, axis) in enumerate(mapping):
        btn = ttk.Button(jog, text=label, width=6)
        btn.grid(row=idx // 4, column=idx % 4, padx=4, pady=4)
        bind_hold_button(
            btn,
            on_press=lambda a=axis: _set_jog(move_api, state, a),
            on_release=lambda: _set_jog(move_api, state, None),
        )

    speed_box = ttk.LabelFrame(outer, text="Speed")
    speed_box.pack(fill="x", pady=6)
    ttk.Scale(speed_box, from_=5, to=60, variable=speed, orient="horizontal").pack(fill="x", padx=8, pady=4)

    def _toggle_suction():
        _set_jog(move_api, state, None)
        state["suction"] = not state["suction"]
        dashboard.ToolDO(SUCTION_DO, 1 if state["suction"] else 0)
        suction_btn.configure(text=f"Suction: {'ON' if state['suction'] else 'OFF'}")

    controls = ttk.Frame(outer)
    controls.pack(fill="x", pady=10)
    suction_btn = ttk.Button(controls, text="Suction: OFF", command=_toggle_suction)
    suction_btn.pack(side="left")
    ttk.Button(controls, text="Home", command=lambda: (_set_jog(move_api, state, None), go_home(move_api))).pack(side="left", padx=8)
    ttk.Button(controls, text="Quit", command=root.destroy).pack(side="right")

    def _tick():
        try:
            dashboard.SpeedFactor(int(speed.get()))
            x, y, z, r = parse_pose(dashboard.GetPose())
            pose_labels["X"].configure(text=f"{x:7.1f} mm")
            pose_labels["Y"].configure(text=f"{y:7.1f} mm")
            pose_labels["Z"].configure(text=f"{z:7.1f} mm")
            pose_labels["R"].configure(text=f"{r:7.1f} deg")
        except Exception:
            pass

    schedule_periodic(root, 120, _tick)
    root.mainloop()
    _set_jog(move_api, state, None)
    if state["suction"]:
        dashboard.ToolDO(SUCTION_DO, 0)


def main():
    parser = argparse.ArgumentParser(description="MG400 GUI/keyboard teleop")
    add_target_arguments(parser)
    parser.add_argument("--mode", choices=("gui", "keyboard"), default="gui", help="Control mode (default: gui)")
    parser.add_argument("--viz", action="store_true", help="Enable visualizer")
    args = parser.parse_args()

    ip, dashboard, move_api, feed = connect_from_args_or_exit(args)
    viz = RobotViz(enabled=args.viz)
    try:
        check_errors(dashboard)
        dashboard.EnableRobot()
        dashboard.SpeedFactor(JOG_SPEED)
        viz.attach(move_api)
        print(f"Connected to {ip}.")
        if args.mode == "keyboard":
            _run_keyboard(move_api, dashboard)
        else:
            _run_gui(move_api, dashboard)
    finally:
        try:
            move_api.MoveJog("")
        except Exception:
            pass
        try:
            viz.close()
        except Exception:
            pass
        try:
            dashboard.DisableRobot()
        except Exception:
            pass
        close_all(dashboard, move_api, feed)
        print("Connections closed.")


if __name__ == "__main__":
    main()
