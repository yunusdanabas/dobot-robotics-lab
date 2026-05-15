"""
07_keyboard_teleop.py — Magician teleop: Tk GUI by default; --mode keyboard for terminal keys.

Run: python 07_keyboard_teleop.py [--mode gui|keyboard] [--no-viz]
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

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# Hold-to-move: jog speed (mm/s or deg/s) while key is held
JOG_VELOCITY_MM = 80   # mm/s for X, Y, Z
JOG_VELOCITY_DEG = 45  # deg/s for R

# Control loop rate (Hz) — target update frequency
LOOP_HZ = 40

# Command dispatch rate: send move_to at most this often to avoid serial flood
# Increased from 15 to 20 now that move_to no longer blocks on get_pose()
CMD_HZ = 20

# Key-release threshold (s): if no key event for this axis for longer, stop motion
RELEASE_THRESHOLD = 0.12


def _run_keyboard(bot, *, go_home, unpack_pose, clamp, safe_bounds):
    x, y, z, r, *_ = unpack_pose(bot.get_pose())
    print("Keyboard mode ready. Hold WASD/arrows for X/Y, R/F for Z, Q/E for rotation.")

    intent = {"x": 0, "y": 0, "z": 0, "r": 0}
    last_key_time = {"x": 0.0, "y": 0.0, "z": 0.0, "r": 0.0}
    dt = 1.0 / LOOP_HZ
    cmd_interval = 1.0 / CMD_HZ
    last_cmd_time = 0.0
    had_intent_prev = False
    suction_on = False

    with TerminalKeyReader() as keys:
        while True:
            now = time.perf_counter()
            key = keys.read_key(timeout_s=dt)
            if key is not None:
                if key == " ":
                    suction_on = not suction_on
                    bot.suck(suction_on)
                elif key == "h":
                    go_home(bot)
                    x, y, z, r, *_ = unpack_pose(bot.get_pose())
                    for ax in intent:
                        intent[ax] = 0
                elif key == "esc":
                    break
                else:
                    if key in ("right", "d"):
                        intent["x"], last_key_time["x"] = 1, now
                    elif key in ("left", "a"):
                        intent["x"], last_key_time["x"] = -1, now
                    elif key in ("up", "w"):
                        intent["y"], last_key_time["y"] = 1, now
                    elif key in ("down", "s"):
                        intent["y"], last_key_time["y"] = -1, now
                    elif key == "r":
                        intent["z"], last_key_time["z"] = 1, now
                    elif key == "f":
                        intent["z"], last_key_time["z"] = -1, now
                    elif key == "q":
                        intent["r"], last_key_time["r"] = 1, now
                    elif key == "e":
                        intent["r"], last_key_time["r"] = -1, now

            for ax in ("x", "y", "z", "r"):
                if intent[ax] != 0 and (now - last_key_time[ax]) > RELEASE_THRESHOLD:
                    intent[ax] = 0

            v_mm = JOG_VELOCITY_MM * dt
            v_r = JOG_VELOCITY_DEG * dt
            x = clamp(x + intent["x"] * v_mm, *safe_bounds["x"])
            y = clamp(y + intent["y"] * v_mm, *safe_bounds["y"])
            z = clamp(z + intent["z"] * v_mm, *safe_bounds["z"])
            r = clamp(r + intent["r"] * v_r, *safe_bounds["r"])

            has_intent = any(intent.values())
            if has_intent and (now - last_cmd_time) >= cmd_interval:
                bot.move_to(x, y, z, r, wait=False)
                last_cmd_time = now
            elif had_intent_prev and not has_intent:
                try:
                    bot._set_queued_cmd_stop_exec()
                    bot._set_queued_cmd_clear()
                    bot._set_queued_cmd_start_exec()
                except Exception:
                    pass
            had_intent_prev = has_intent
            print(f"  X={x:.1f}  Y={y:.1f}  Z={z:.1f}  R={r:.1f}", end="\r", flush=True)

            elapsed = time.perf_counter() - now
            if elapsed < dt:
                time.sleep(dt - elapsed)

    if suction_on:
        bot.suck(False)
    print()


def _run_gui(bot, *, go_home, unpack_pose, clamp, safe_bounds):
    root = tk.Tk()
    root.title("Magician Teleop")
    root.geometry("560x420")

    pose = {"x": 0.0, "y": 0.0, "z": 0.0, "r": 0.0}
    intent = {"x": 0, "y": 0, "z": 0, "r": 0}
    state = {"suction": False}

    vel_mm = tk.DoubleVar(value=JOG_VELOCITY_MM)
    vel_deg = tk.DoubleVar(value=JOG_VELOCITY_DEG)

    frame = ttk.Frame(root, padding=12)
    frame.pack(fill="both", expand=True)
    ttk.Label(frame, text="Magician GUI Teleop", font=("TkDefaultFont", 12, "bold")).pack(anchor="w")

    for axis in ("X", "Y", "Z", "R"):
        row, value = labeled_value(frame, f"{axis}:")
        row.pack(anchor="w")
        pose[axis.lower()] = value

    jog = ttk.LabelFrame(frame, text="Jog (hold button)")
    jog.pack(fill="x", pady=10)
    mapping = [
        ("X-", "x", -1), ("X+", "x", 1), ("Y-", "y", -1), ("Y+", "y", 1),
        ("Z-", "z", -1), ("Z+", "z", 1), ("R-", "r", -1), ("R+", "r", 1),
    ]
    for idx, (label, ax, sign) in enumerate(mapping):
        btn = ttk.Button(jog, text=label, width=6)
        btn.grid(row=idx // 4, column=idx % 4, padx=4, pady=4)
        bind_hold_button(
            btn,
            on_press=lambda a=ax, s=sign: intent.__setitem__(a, s),
            on_release=lambda a=ax: intent.__setitem__(a, 0),
        )

    cfg = ttk.LabelFrame(frame, text="Speeds")
    cfg.pack(fill="x", pady=6)
    ttk.Label(cfg, text="Linear mm/s").grid(row=0, column=0, sticky="w")
    ttk.Scale(cfg, from_=20, to=150, variable=vel_mm, orient="horizontal").grid(row=0, column=1, sticky="ew", padx=8)
    ttk.Label(cfg, text="Rotation deg/s").grid(row=1, column=0, sticky="w")
    ttk.Scale(cfg, from_=10, to=120, variable=vel_deg, orient="horizontal").grid(row=1, column=1, sticky="ew", padx=8)
    cfg.columnconfigure(1, weight=1)

    actions = ttk.Frame(frame)
    actions.pack(fill="x", pady=10)
    suction_btn = ttk.Button(actions, text="Suction: OFF")
    suction_btn.pack(side="left")

    def _toggle_suction() -> None:
        state["suction"] = not state["suction"]
        bot.suck(state["suction"])
        suction_btn.configure(text=f"Suction: {'ON' if state['suction'] else 'OFF'}")

    suction_btn.configure(command=_toggle_suction)
    ttk.Button(actions, text="Home", command=lambda: go_home(bot)).pack(side="left", padx=8)
    ttk.Button(actions, text="Quit", command=root.destroy).pack(side="right")

    dt = 1.0 / LOOP_HZ
    cmd_interval = 1.0 / CMD_HZ
    last_cmd = {"t": 0.0}

    def _update() -> None:
        now = time.perf_counter()
        try:
            x, y, z, r, *_ = unpack_pose(bot.get_pose())
        except Exception:
            x = y = z = r = 0.0
        pose["x"].configure(text=f"{x:7.1f} mm")
        pose["y"].configure(text=f"{y:7.1f} mm")
        pose["z"].configure(text=f"{z:7.1f} mm")
        pose["r"].configure(text=f"{r:7.1f} deg")
        if any(intent.values()) and (now - last_cmd["t"]) >= cmd_interval:
            x = clamp(x + intent["x"] * vel_mm.get() * dt, *safe_bounds["x"])
            y = clamp(y + intent["y"] * vel_mm.get() * dt, *safe_bounds["y"])
            z = clamp(z + intent["z"] * vel_mm.get() * dt, *safe_bounds["z"])
            r = clamp(r + intent["r"] * vel_deg.get() * dt, *safe_bounds["r"])
            bot.move_to(x, y, z, r, wait=False)
            last_cmd["t"] = now

    schedule_periodic(root, int(1000 / LOOP_HZ), _update)
    root.mainloop()
    if state["suction"]:
        bot.suck(False)


def main():
    parser = argparse.ArgumentParser(description="Magician GUI/keyboard teleop")
    parser.add_argument("--mode", choices=("gui", "keyboard"), default="gui", help="Control mode (default: gui)")
    parser.add_argument("--no-viz", action="store_true", help="Disable real-time visualization")
    args = parser.parse_args()

    from pydobotplus import Dobot
    from utils import clamp, find_port, go_home, prepare_robot, SAFE_ACCELERATION, SAFE_BOUNDS, SAFE_VELOCITY, unpack_pose
    from viz import RobotViz

    port = find_port()
    if port is None:
        sys.exit("[Error] No serial port found. Run 01_find_port.py first.")
    if args.mode == "keyboard" and not TerminalKeyReader.require_tty():
        sys.exit("[Error] Keyboard mode requires an interactive terminal (TTY).")

    bot = Dobot(port=port)
    bot.speed(SAFE_VELOCITY, SAFE_ACCELERATION)
    prepare_robot(bot)
    viz = RobotViz(enabled=not args.no_viz)
    viz.attach(bot)
    print(f"Connected on {port}")
    try:
        if args.mode == "keyboard":
            _run_keyboard(bot, go_home=go_home, unpack_pose=unpack_pose, clamp=clamp, safe_bounds=SAFE_BOUNDS)
        else:
            _run_gui(bot, go_home=go_home, unpack_pose=unpack_pose, clamp=clamp, safe_bounds=SAFE_BOUNDS)
    finally:
        viz.close()
        bot.close()


if __name__ == "__main__":
    main()
