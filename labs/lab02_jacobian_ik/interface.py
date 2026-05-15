"""
Lab 02 - Jacobian and inverse kinematics (ME403, Sabanci University).

Tk GUI by default; use --mode terminal for a text REPL. Both call myCode.run(robot).

Robots: DOBOT_ROBOT_TYPE magician or mg400; simulation defaults on. See student_api.md
for env vars (DOBOT_SIMULATION, DOBOT_SIM_BACKEND, DOBOT_EE, ...).
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import myCode
import utils as U


def _parse_move_command(line: str) -> list[float] | None:
    parts = line.split()
    if len(parts) == 5 and parts[0].lower() == "move":
        values = parts[1:]
    elif len(parts) == 4:
        values = parts
    else:
        return None
    try:
        return [float(value) for value in values]
    except ValueError:
        return None


def run_interface(
    *,
    execute_callback: Callable[[object], None],
    execute_prompt: str,
    execute_banner: str,
    help_tokens: Sequence[str] = ("move q1 q2 q3 q4", "q", "x"),
) -> None:
    """Terminal REPL: 'move q1..q4' jogs the robot; 'x' calls execute_callback(robot)."""
    robot = U.setup()
    try:
        print("\nEnter: 'move q1 q2 q3 q4' to move the robot once")
        print("Type 'q' to quit.")
        print(f"Type 'x' to {execute_prompt}.\n")

        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue
            if line.lower() in ("q", "quit"):
                break
            if line.lower() in ("x", "execute"):
                print(execute_banner)
                execute_callback(robot)
                continue

            q = _parse_move_command(line)
            if q is None:
                print(f"  Use: {' | '.join(help_tokens)}")
                continue

            x, y, z, r = U.move_and_get_feedback(robot, q)
            print(f" Pose: X={x:.2f} Y={y:.2f} Z={z:.2f} R={r:.2f}")
    finally:
        U.teardown(robot)


def run_interface_gui(
    *,
    execute_callback: Callable[[object], None],
    execute_prompt: str,
    execute_banner: str,
) -> None:
    """Tkinter GUI: joint fields + 'Move Once' / 'Run Lab Code' buttons."""
    try:
        import tkinter as tk
        from tkinter import messagebox, ttk
    except ImportError as exc:
        print(f"[gui] tkinter not available ({exc}). Use --mode terminal.")
        return

    robot = U.setup()
    root = tk.Tk()
    root.title("Lab Interface GUI")
    root.geometry("520x340")

    q_vars = [tk.StringVar(value="0") for _ in range(4)]
    status = tk.StringVar(value="Ready.")

    outer = ttk.Frame(root, padding=12)
    outer.pack(fill="both", expand=True)
    ttk.Label(outer, text="GUI Robot Control", font=("TkDefaultFont", 12, "bold")).pack(anchor="w")
    ttk.Label(outer, text=f"Custom action: {execute_prompt}").pack(anchor="w", pady=(2, 8))

    grid = ttk.Frame(outer)
    grid.pack(fill="x")
    for i, name in enumerate(("Q1", "Q2", "Q3", "Q4")):
        ttk.Label(grid, text=name).grid(row=0, column=i, padx=4, sticky="w")
        ttk.Entry(grid, textvariable=q_vars[i], width=10).grid(row=1, column=i, padx=4, pady=4)

    def _move_once() -> None:
        try:
            q = [float(v.get()) for v in q_vars]
            x, y, z, r = U.move_and_get_feedback(robot, q)
            status.set(f"Pose: X={x:.2f} Y={y:.2f} Z={z:.2f} R={r:.2f}")
        except Exception as exc:
            messagebox.showerror("Move Error", str(exc))

    def _execute() -> None:
        status.set(execute_banner)
        root.update_idletasks()   # flush pending UI repaints before the callback blocks the thread
        try:
            execute_callback(robot)
            status.set(f"{execute_banner} done.")
        except Exception as exc:
            messagebox.showerror("Execution Error", str(exc))

    controls = ttk.Frame(outer)
    controls.pack(fill="x", pady=10)
    ttk.Button(controls, text="Move Once", command=_move_once).pack(side="left")
    ttk.Button(controls, text="Run Lab Code", command=_execute).pack(side="left", padx=8)
    ttk.Button(controls, text="Quit", command=root.destroy).pack(side="right")

    ttk.Label(outer, textvariable=status, wraplength=480).pack(anchor="w", pady=8)

    try:
        root.mainloop()
    finally:
        U.teardown(robot)


def main() -> None:
    """Entry point: dispatches to GUI (default) or terminal based on --mode."""
    import argparse

    parser = argparse.ArgumentParser(description="Lab 02 interface (GUI-first)")
    parser.add_argument("--mode", choices=("gui", "terminal"), default="gui", help="Control mode (default: gui)")
    args = parser.parse_args()

    kwargs = dict(
        execute_callback=myCode.run,
        execute_prompt="execute Lab 2 code",
        execute_banner="Running Lab 2 code...",
    )
    if args.mode == "terminal":
        run_interface(**kwargs)
    else:
        run_interface_gui(**kwargs)


if __name__ == "__main__":
    main()
