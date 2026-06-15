"""Magician URDF teleop with MuJoCo (Tk, keyboard, optional joystick). No ROS.

Run from dobot-robotics-lab package root. Needs mujoco. Full key and gamepad maps
come from --help, H in keyboard mode, and argparse epilog.
"""

from __future__ import annotations

import argparse
import os
import random
import sys
import threading
import time
from pathlib import Path

import tkinter as tk
from tkinter import ttk

_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from terminal_keys import TerminalKeyReader
from simulation.runtime.collision import CollisionGuardConfig, CollisionContact, contact_summaries, should_ignore_contact
from simulation.runtime.kinematics import fk_magician
from simulation.runtime.magician_joint_mapping import firmware_deg_to_visual_rad
from simulation.runtime.urdf_paths import resolve_magician_urdf_path

# ---------------------------------------------------------------------------
# MuJoCo viewer backend
# ---------------------------------------------------------------------------

class MagicianMuJoCoViewer:
    """Lightweight MuJoCo wrapper that only visualises joint angles."""

    JOINT_BASE = "magician_joint_1"
    JOINT_SHOULDER = "magician_joint_2"
    JOINT_FOREARM = "magician_joint_3"
    JOINT_WRIST = "magician_joint_4"
    JOINT_MIMIC_1 = "magician_joint_mimic_1"
    JOINT_MIMIC_2 = "magician_joint_mimic_2"

    JOINT_LIMITS_DEG = {
        "j1": (-90.0, 90.0),
        "j2": (0.0, 90.0),
        "j3": (-40.0, 90.0),
    }

    def __init__(
        self,
        collision_guard: bool | None = None,
        tool: str = "none",
    ):
        if tool not in ("none", "motor", "suction"):
            raise ValueError(f"Unknown tool {tool!r}; expected 'none', 'motor', or 'suction'")
        self._tool = tool
        self._collision_config = CollisionGuardConfig.from_value(collision_guard)
        self._collision_stopped = False
        self._last_collision: CollisionContact | None = None
        os.environ.setdefault("MUJOCO_GL", "egl")
        _here = Path(__file__).resolve().parent
        _urdf = resolve_magician_urdf_path(tool=tool)
        try:
            import mujoco
        except ImportError as exc:
            raise ImportError("MuJoCo backend requires: pip install mujoco") from exc

        sys.path.insert(0, str(_here.parent / "runtime"))
        from urdf_loader import prepare_urdf_for_mujoco

        cache_dir = os.environ.get(
            "DOBOT_SIM_CACHE", os.path.join(os.path.expanduser("~"), ".cache", "dobot_sim")
        )
        wrapper = prepare_urdf_for_mujoco(_urdf, cache_dir)

        self._mj = mujoco
        self._model = mujoco.MjModel.from_xml_path(str(wrapper))
        self._data = mujoco.MjData(self._model)
        self._viewer = None
        self._viewer_mod = None

        self._qposadr = {}
        for name in (self.JOINT_BASE, self.JOINT_SHOULDER, self.JOINT_FOREARM,
                     self.JOINT_WRIST, self.JOINT_MIMIC_1, self.JOINT_MIMIC_2):
            self._qposadr[name] = int(self._model.joint(name).qposadr[0])
        self._body_names = {
            i: (self._mj.mj_id2name(self._model, self._mj.mjtObj.mjOBJ_BODY, i) or str(i))
            for i in range(self._model.nbody)
        }
        ignored_pairs = {
            tuple(sorted((self._body_names.get(int(parent), str(parent)), self._body_names.get(i, str(i)))))
            for i, parent in enumerate(self._model.body_parentid)
            if i != 0
        }
        ignored_pairs.update(self._collision_config.ignored_link_pairs)
        ignored_pairs.update({
            tuple(sorted(pair))
            for pair in (
                ("link_1", "link_5"),
            )
        })
        self._collision_config = CollisionGuardConfig(
            enabled=self._collision_config.enabled,
            stop_on_collision=self._collision_config.stop_on_collision,
            check_self=self._collision_config.check_self,
            check_environment=self._collision_config.check_environment,
            distance_threshold=self._collision_config.distance_threshold,
            ignored_name_prefixes=self._collision_config.ignored_name_prefixes,
            ignored_link_pairs=frozenset(ignored_pairs),
        )

        self._viewer_mod = self._import_mujoco_viewer()
        self._viewer = self._viewer_mod.launch_passive(self._model, self._data)
        self._show_visual_geoms_only(self._viewer)

        self._lock = threading.Lock()
        self._joint_deg = [0.0, 0.0, 0.0]
        self._last_safe_joint_deg = list(self._joint_deg)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    @staticmethod
    def _import_mujoco_viewer():
        try:
            import mujoco.viewer as mj_viewer
        except ImportError as exc:
            raise ImportError("MuJoCo viewer is unavailable. Ensure `mujoco.viewer` can be imported.") from exc
        return mj_viewer

    @staticmethod
    def _show_visual_geoms_only(viewer) -> None:
        """Hide MuJoCo collision proxies in the viewer without disabling physics."""
        try:
            viewer.opt.geomgroup[0] = 0
            viewer.opt.geomgroup[1] = 1
        except Exception:
            pass

    def _apply_visual_joints(self, q_deg: list[float], *, forward: bool = False) -> None:
        # MuJoCo does not auto-propagate equality constraints; mimic joints must be set explicitly.
        # forward=True runs full dynamics (required before reading contact data).
        j1, j2, j3 = q_deg
        mapped = firmware_deg_to_visual_rad(
            j1,
            j2,
            j3,
        )
        self._data.qpos[self._qposadr[self.JOINT_BASE]]     = mapped.joint_1
        self._data.qpos[self._qposadr[self.JOINT_SHOULDER]] = mapped.joint_2
        self._data.qpos[self._qposadr[self.JOINT_FOREARM]]  = mapped.joint_3
        self._data.qpos[self._qposadr[self.JOINT_WRIST]]    = mapped.joint_4
        self._data.qpos[self._qposadr[self.JOINT_MIMIC_1]]  = mapped.joint_mimic_1
        self._data.qpos[self._qposadr[self.JOINT_MIMIC_2]]  = mapped.joint_mimic_2
        if forward:
            self._mj.mj_forward(self._model, self._data)
        else:
            self._mj.mj_kinematics(self._model, self._data)

    def _collect_collision_contacts(self) -> list[CollisionContact]:
        if not self._collision_config.enabled or not self._collision_config.check_self:
            return []
        contacts = []
        for i in range(int(self._data.ncon)):
            c = self._data.contact[i]
            body_a = int(self._model.geom_bodyid[c.geom1])
            body_b = int(self._model.geom_bodyid[c.geom2])
            name_a = self._body_names.get(body_a, str(body_a))
            name_b = self._body_names.get(body_b, str(body_b))
            if should_ignore_contact(name_a, name_b, self._collision_config):
                continue
            contacts.append(
                CollisionContact(
                    backend="mujoco",
                    kind="self",
                    body_a="magician",
                    body_b="magician",
                    link_a=name_a,
                    link_b=name_b,
                    distance=float(c.dist),
                )
            )
        return contacts

    def _sync_viewer_overlay(self, q_deg: list[float]) -> None:
        if self._viewer is None or not self._viewer.is_running():
            return
        x, y, z, r = fk_magician(q_deg, tool=self._tool)
        text = f"X={x:.1f}  Y={y:.1f}  Z={z:.1f}  R=0.0"
        if self._collision_stopped and self._last_collision is not None:
            text += f"\nCOLLISION STOP: {self._last_collision.link_a}/{self._last_collision.link_b}"
        try:
            if hasattr(self._viewer, "set_texts"):
                self._viewer.set_texts([
                    (
                        self._mj.mjtFontScale.mjFONTSCALE_150,
                        self._mj.mjtGridPos.mjGRID_TOPLEFT,
                        "EE Pose",
                        text,
                    )
                ])
            self._viewer.sync()
        except Exception:
            pass

    def _write_joints(self) -> None:
        q_candidate = list(self._joint_deg)
        self._apply_visual_joints(q_candidate, forward=self._collision_config.enabled)
        contacts = self._collect_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._collision_stopped = True
            self._last_collision = contacts[0]
            self._joint_deg = list(self._last_safe_joint_deg)  # revert to last safe pose
            self._apply_visual_joints(self._joint_deg, forward=True)
            print(f"\n[MagicianMuJoCo] COLLISION STOP: {contact_summaries(contacts, limit=1)}")
        else:
            self._collision_stopped = False
            self._last_collision = None
            self._last_safe_joint_deg = q_candidate  # record safe pose for future reverts
        self._sync_viewer_overlay(self._joint_deg)

    def _update_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                self._write_joints()
            time.sleep(1.0 / 20.0)

    def home(self) -> None:
        with self._lock:
            self._joint_deg = [0.0, 0.0, 0.0]

    def randomize(self) -> None:
        with self._lock:
            self._joint_deg = [
                random.uniform(*self.JOINT_LIMITS_DEG["j1"]),
                random.uniform(*self.JOINT_LIMITS_DEG["j2"]),
                random.uniform(*self.JOINT_LIMITS_DEG["j3"]),
            ]

    def get_joint_deg(self) -> list[float]:
        with self._lock:
            return list(self._joint_deg)

    def get_ee_pose(self) -> tuple[float, float, float, float]:
        j1, j2, j3 = self.get_joint_deg()
        return fk_magician([j1, j2, j3, 0.0], tool=self._tool)

    def collision_summary(self) -> str:
        with self._lock:
            if self._last_collision is None:
                return "Collision: OK" if self._collision_config.enabled else "Collision: OFF"
            return f"Collision: STOP {self._last_collision.link_a}/{self._last_collision.link_b}"

    def clear_collision_stop(self) -> None:
        with self._lock:
            self._collision_stopped = False
            self._last_collision = None

    def set_joint_deg(self, index: int, value_deg: float) -> None:
        joint_name = ("j1", "j2", "j3")[index]
        lo, hi = self.JOINT_LIMITS_DEG[joint_name]
        with self._lock:
            self._joint_deg[index] = max(lo, min(hi, float(value_deg)))

    def nudge_deg(self, index: int, delta_deg: float) -> None:
        with self._lock:
            joint_name = ("j1", "j2", "j3")[index]
            lo, hi = self.JOINT_LIMITS_DEG[joint_name]
            self._joint_deg[index] = max(lo, min(hi, self._joint_deg[index] + delta_deg))

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None
        self._data = None
        self._model = None


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

GUI_STEP_CHOICES = ("0.5", "1", "2", "5", "10")
DEFAULT_STEP_DEG = 2.0


class MagicianGui:
    def __init__(self, viewer: MagicianMuJoCoViewer, step_deg: float) -> None:
        self._viewer = viewer
        self._root = tk.Tk()
        self._root.title("Magician MuJoCo Teleop")
        self._root.geometry("760x440")
        self._root.minsize(760, 400)
        self._root.bind("<Escape>", lambda _event: self._root.destroy())
        self._root.bind("<c>", lambda _event: self._home())
        self._root.bind("<r>", lambda _event: self._randomize())

        self._step_var = tk.StringVar(value=f"{step_deg:g}")
        self._slider_vars: list[tk.DoubleVar] = []
        self._value_labels: list[ttk.Label] = []
        self._ee_labels: dict[str, ttk.Label] = {}
        self._collision_label: ttk.Label | None = None

        outer = ttk.Frame(self._root, padding=12)
        outer.pack(fill="both", expand=True)

        header = ttk.Label(
            outer,
            text="Magician MuJoCo Joint Teleop",
            font=("TkDefaultFont", 13, "bold"),
        )
        header.pack(anchor="w")

        helper = ttk.Label(outer, text="J1/J2/J3 are controls. End-effector rotation is fixed.")
        helper.pack(anchor="w", pady=(0, 8))

        controls = ttk.Frame(outer)
        controls.pack(fill="x", pady=(0, 8))
        ttk.Label(controls, text="Step size (deg)").pack(side="left")
        step_combo = ttk.Combobox(
            controls,
            textvariable=self._step_var,
            values=GUI_STEP_CHOICES,
            width=6,
            state="readonly",
        )
        step_combo.pack(side="left", padx=(8, 16))
        ttk.Button(controls, text="Home", command=self._home).pack(side="left", padx=(0, 8))
        ttk.Button(controls, text="Random", command=self._randomize).pack(side="left")

        for idx, joint_name in enumerate(("J1", "J2", "J3")):
            row = ttk.Frame(outer)
            row.pack(fill="x", pady=6)

            ttk.Label(row, text=joint_name, width=4).pack(side="left")
            ttk.Button(row, text="-", width=4, command=lambda i=idx: self._nudge(i, -1.0)).pack(side="left")

            joint_key = ("j1", "j2", "j3")[idx]
            lo, hi = MagicianMuJoCoViewer.JOINT_LIMITS_DEG[joint_key]
            var = tk.DoubleVar(value=0.0)
            scale = tk.Scale(
                row,
                from_=lo,
                to=hi,
                orient="horizontal",
                resolution=0.5,
                showvalue=False,
                variable=var,
                command=lambda value, i=idx: self._set_joint(i, value),
                length=420,
            )
            scale.pack(side="left", padx=8, fill="x", expand=True)

            ttk.Button(row, text="+", width=4, command=lambda i=idx: self._nudge(i, +1.0)).pack(side="left", padx=(0, 8))

            value_label = ttk.Label(row, text="0.0 deg", width=10)
            value_label.pack(side="left")

            self._slider_vars.append(var)
            self._value_labels.append(value_label)

        ee_frame = ttk.Frame(outer)
        ee_frame.pack(fill="x", pady=(10, 0))
        for name, unit in (("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("R", "deg")):
            ttk.Label(ee_frame, text=name, width=3).pack(side="left")
            label = ttk.Label(ee_frame, text=f"0.0 {unit}", width=12)
            label.pack(side="left", padx=(0, 10))
            self._ee_labels[name] = label

        self._collision_label = ttk.Label(outer, text="Collision: OFF")
        self._collision_label.pack(anchor="w", pady=(6, 0))

        footer = ttk.Label(
            outer,
            text="Esc closes the window. C homes. R picks a random valid pose.",
        )
        footer.pack(anchor="w", pady=(10, 0))

    def _current_step(self) -> float:
        try:
            return float(self._step_var.get())
        except ValueError:
            return DEFAULT_STEP_DEG

    def _set_joint(self, index: int, value: str) -> None:
        value_deg = float(value)
        self._viewer.set_joint_deg(index, value_deg)
        self._sync_labels()

    def _nudge(self, index: int, direction: float) -> None:
        self._viewer.nudge_deg(index, direction * self._current_step())
        self._sync_from_controller()

    def _home(self) -> None:
        self._viewer.home()
        self._sync_from_controller()

    def _randomize(self) -> None:
        self._viewer.randomize()
        self._sync_from_controller()

    def _sync_labels(self) -> None:
        for idx, value_deg in enumerate(self._viewer.get_joint_deg()):
            self._value_labels[idx].configure(text=f"{value_deg:6.1f} deg")
        x, y, z, r = self._viewer.get_ee_pose()
        self._ee_labels["X"].configure(text=f"{x:7.1f} mm")
        self._ee_labels["Y"].configure(text=f"{y:7.1f} mm")
        self._ee_labels["Z"].configure(text=f"{z:7.1f} mm")
        self._ee_labels["R"].configure(text=f"{r:7.1f} deg")
        if self._collision_label is not None:
            self._collision_label.configure(text=self._viewer.collision_summary())

    def _sync_from_controller(self) -> None:
        for var, value_deg in zip(self._slider_vars, self._viewer.get_joint_deg()):
            var.set(value_deg)
        self._sync_labels()

    def run(self) -> None:
        self._sync_from_controller()
        self._root.mainloop()


# ---------------------------------------------------------------------------
# Keyboard loop
# ---------------------------------------------------------------------------

KEY_TO_DELTA = {
    "left": (0, -1.0),
    "a": (0, -1.0),
    "right": (0, +1.0),
    "d": (0, +1.0),
    "down": (1, -1.0),
    "s": (1, -1.0),
    "up": (1, +1.0),
    "w": (1, +1.0),
    "j": (2, -1.0),
    "l": (2, +1.0),
}


def keyboard_loop(viewer: MagicianMuJoCoViewer, step_deg: float) -> None:
    if not TerminalKeyReader.require_tty():
        raise SystemExit("Keyboard mode requires an interactive terminal (TTY).")

    print(__doc__)
    print("Keyboard mode ready. Hold a key to repeat via terminal auto-repeat.\n")

    current_step = float(step_deg)
    last_status = 0.0
    status_interval = 0.1

    def print_status(force: bool = False) -> None:
        nonlocal last_status
        now = time.perf_counter()
        if not force and (now - last_status) < status_interval:
            return
        j1, j2, j3 = viewer.get_joint_deg()
        x, y, z, r = viewer.get_ee_pose()
        print(
            f"  J1={j1:6.1f} J2={j2:6.1f} J3={j3:6.1f} "
            f"| X={x:6.1f} Y={y:6.1f} Z={z:6.1f} R={r:6.1f} "
            f"| {viewer.collision_summary()} | Step={current_step:5.2f} deg",
            end="\r",
        )
        last_status = now

    with TerminalKeyReader() as keys:
        print_status(force=True)
        while True:
            key = keys.read_key(timeout_s=0.05)
            if key is None:
                print_status()
                continue

            if key == "esc":
                print("\nQuitting keyboard teleop.")
                break
            if key in ("0", "c"):
                viewer.home()
            elif key == "r":
                viewer.randomize()
            elif key == "[":
                current_step = max(0.25, current_step / 2.0)
            elif key == "]":
                current_step = min(20.0, current_step * 2.0)
            elif key == "h":
                print("\n")
                print(__doc__)
            elif key in KEY_TO_DELTA:
                joint_idx, sign = KEY_TO_DELTA[key]
                viewer.nudge_deg(joint_idx, sign * current_step)
            print_status(force=True)

    print()


def joystick_loop(
    viewer: MagicianMuJoCoViewer,
    *,
    axis_speed_deg_s: float,
    deadzone: float,
    joystick_index: int,
) -> None:
    try:
        import pygame
    except ImportError as exc:
        raise SystemExit(
            "Joystick mode requires pygame.\n"
            "Install with: pip install pygame"
        ) from exc

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= joystick_index:
        raise SystemExit(
            f"No joystick found at index {joystick_index}. "
            f"Detected: {pygame.joystick.get_count()}"
        )
    stick = pygame.joystick.Joystick(joystick_index)
    stick.init()

    print(__doc__)
    print(f"Joystick mode ready: {stick.get_name()} (index={joystick_index})")
    print("Back button exits, A homes, Y randomizes.\n")

    # Common button IDs on most XInput controllers.
    BTN_A = 0
    BTN_BACK = 6
    BTN_Y = 3

    last_status = 0.0
    status_interval = 0.1
    prev_buttons = [False] * max(16, stick.get_numbuttons())
    prev_t = time.perf_counter()
    speed = max(1.0, float(axis_speed_deg_s))
    dz = max(0.0, min(0.5, float(deadzone)))

    def axis_value(axis_id: int) -> float:
        if axis_id >= stick.get_numaxes():
            return 0.0
        v = float(stick.get_axis(axis_id))
        return 0.0 if abs(v) < dz else v

    def button_pressed_once(button_id: int) -> bool:
        if button_id >= stick.get_numbuttons():
            return False
        current = bool(stick.get_button(button_id))
        previous = prev_buttons[button_id]
        prev_buttons[button_id] = current
        return current and not previous

    def print_status(force: bool = False) -> None:
        nonlocal last_status
        now = time.perf_counter()
        if not force and (now - last_status) < status_interval:
            return
        j1, j2, j3 = viewer.get_joint_deg()
        x, y, z, r = viewer.get_ee_pose()
        print(
            f"  J1={j1:6.1f} J2={j2:6.1f} J3={j3:6.1f} "
            f"| X={x:6.1f} Y={y:6.1f} Z={z:6.1f} R={r:6.1f} "
            f"| {viewer.collision_summary()} | Speed={speed:5.1f} deg/s",
            end="\r",
        )
        last_status = now

    try:
        print_status(force=True)
        while True:
            pygame.event.pump()
            now = time.perf_counter()
            dt = min(0.1, max(0.0, now - prev_t))
            prev_t = now

            # Left stick: X->J1, Y->J2 (invert Y so up is positive)
            # Right stick: Y->J3 (invert)
            v_j1 = axis_value(0)
            v_j2 = -axis_value(1)
            v_j3 = -axis_value(3)

            if v_j1:
                viewer.nudge_deg(0, v_j1 * speed * dt)
            if v_j2:
                viewer.nudge_deg(1, v_j2 * speed * dt)
            if v_j3:
                viewer.nudge_deg(2, v_j3 * speed * dt)

            if button_pressed_once(BTN_A):
                viewer.home()
            if button_pressed_once(BTN_Y):
                viewer.randomize()
            if button_pressed_once(BTN_BACK):
                print("\nQuitting joystick teleop.")
                break

            print_status()
            time.sleep(0.01)
    finally:
        print()
        try:
            stick.quit()
        except Exception:
            pass
        pygame.joystick.quit()
        pygame.quit()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Magician MuJoCo teleop")
    parser.add_argument(
        "--mode",
        choices=("gui", "keyboard", "joystick"),
        default="gui",
        help="Control mode (default: gui)",
    )
    parser.add_argument(
        "--step-deg",
        type=float,
        default=DEFAULT_STEP_DEG,
        help=f"Initial step size in degrees (default: {DEFAULT_STEP_DEG})",
    )
    parser.add_argument(
        "--joystick-speed-deg-s",
        type=float,
        default=120.0,
        help="Joystick max angular speed per joint in deg/s (default: 120)",
    )
    parser.add_argument(
        "--joystick-deadzone",
        type=float,
        default=0.15,
        help="Joystick axis deadzone in [0..0.5] (default: 0.15)",
    )
    parser.add_argument(
        "--joystick-index",
        type=int,
        default=0,
        help="Joystick device index for pygame (default: 0)",
    )
    parser.add_argument(
        "--collision-guard",
        action="store_true",
        default=None,
        help="Stop and revert when simulator self-collision is detected (default: DOBOT_COLLISION_GUARD or off)",
    )
    parser.add_argument(
        "--ee-mode",
        choices=("none", "motor", "suction"),
        default=os.environ.get("DOBOT_EE", "none"),
        help="End-effector mode (none: bare default flange; motor: +60mm X legacy mode; "
             "suction: +60mm X and -70mm Z TCP offset (physical cup tip). "
             "Falls back to env var DOBOT_EE.",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    viewer = MagicianMuJoCoViewer(
        collision_guard=args.collision_guard,
        tool=args.ee_mode,
    )
    viewer.home()
    try:
        if args.mode == "keyboard":
            keyboard_loop(viewer, step_deg=args.step_deg)
        elif args.mode == "joystick":
            joystick_loop(
                viewer,
                axis_speed_deg_s=args.joystick_speed_deg_s,
                deadzone=args.joystick_deadzone,
                joystick_index=args.joystick_index,
            )
        else:
            try:
                MagicianGui(viewer, step_deg=args.step_deg).run()
            except tk.TclError as exc:
                raise SystemExit(
                    "GUI mode could not open a display. "
                    "Use --mode keyboard in a terminal instead.\n"
                    f"Tk error: {exc}"
                ) from exc
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
