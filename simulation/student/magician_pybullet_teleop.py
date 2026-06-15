"""Magician URDF teleop with PyBullet (Tk panel or keyboard). No ROS.

Run from dobot-robotics-lab package root so terminal_keys resolves (see simulation/student/README).
Requires pybullet. Keymaps print from --help, H in keyboard mode, and the argparse epilog.
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
# PyBullet viewer backend
# ---------------------------------------------------------------------------

class MagicianPyBulletViewer:
    """Lightweight PyBullet wrapper that only visualises joint angles."""

    JOINT_BASE = "magician_joint_1"
    JOINT_SHOULDER = "magician_joint_2"
    JOINT_FOREARM = "magician_joint_3"
    JOINT_WRIST = "magician_joint_4"
    JOINT_MIMIC_1 = "magician_joint_mimic_1"
    JOINT_MIMIC_2 = "magician_joint_mimic_2"
    BASE_LINK = "magician_root_link"

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
        _here = Path(__file__).resolve().parent
        _urdf = resolve_magician_urdf_path(tool=tool)
        try:
            import pybullet as p
            import pybullet_data
        except Exception as exc:
            raise ImportError("PyBullet backend requires: pip install pybullet") from exc

        sys.path.insert(0, str(_here.parent / "runtime"))
        from urdf_loader import prepare_urdf_for_pybullet

        cache_dir = os.environ.get(
            "DOBOT_SIM_CACHE", os.path.join(os.path.expanduser("~"), ".cache", "dobot_sim")
        )
        _urdf_prepared = prepare_urdf_for_pybullet(_urdf, cache_dir)

        self._p = p
        self._client = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self._client)
        self._plane_id = p.loadURDF("plane.urdf", physicsClientId=self._client)
        flags = p.URDF_MAINTAIN_LINK_ORDER
        if self._collision_config.enabled:
            flags |= getattr(p, "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT", p.URDF_USE_SELF_COLLISION)
        self._robot_id = p.loadURDF(
            str(_urdf_prepared),
            useFixedBase=True,
            flags=flags,
            physicsClientId=self._client,
        )

        self._joint_idx = {}
        n = p.getNumJoints(self._robot_id, physicsClientId=self._client)
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            self._joint_idx[info[1].decode()] = i
        self._link_names = {-1: self.BASE_LINK}
        self._parent_links: dict[int, int] = {}
        for i in range(n):
            info = p.getJointInfo(self._robot_id, i, physicsClientId=self._client)
            self._link_names[i] = info[12].decode() or info[1].decode()
            self._parent_links[i] = int(info[16])
        ignored_pairs = {
            tuple(sorted((self._link_names.get(parent, str(parent)), self._link_names.get(child, str(child)))))
            for child, parent in self._parent_links.items()
        }
        ignored_pairs.update(self._collision_config.ignored_link_pairs)
        self._collision_config = CollisionGuardConfig(
            enabled=self._collision_config.enabled,
            stop_on_collision=self._collision_config.stop_on_collision,
            check_self=self._collision_config.check_self,
            check_environment=self._collision_config.check_environment,
            distance_threshold=self._collision_config.distance_threshold,
            ignored_name_prefixes=self._collision_config.ignored_name_prefixes,
            ignored_link_pairs=frozenset(ignored_pairs),
        )

        self._lock = threading.Lock()
        self._joint_deg = [0.0, 0.0, 0.0]
        self._last_safe_joint_deg = list(self._joint_deg)
        self._debug_text_id = -1
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _apply_visual_joints(self, q_deg: list[float]) -> None:
        p = self._p
        cid = self._client
        j1, j2, j3 = q_deg
        mapped = firmware_deg_to_visual_rad(
            j1,
            j2,
            j3,
        )
        mapping = {
            self.JOINT_BASE:     mapped.joint_1,
            self.JOINT_SHOULDER: mapped.joint_2,
            self.JOINT_FOREARM:  mapped.joint_3,
            self.JOINT_WRIST:    mapped.joint_4,
            self.JOINT_MIMIC_1:  mapped.joint_mimic_1,
            self.JOINT_MIMIC_2:  mapped.joint_mimic_2,
        }
        for name, angle in mapping.items():
            idx = self._joint_idx.get(name)
            if idx is not None:
                p.resetJointState(self._robot_id, idx, angle, physicsClientId=cid)
        p.stepSimulation(physicsClientId=cid)

    def _collect_collision_contacts(self) -> list[CollisionContact]:
        if not self._collision_config.enabled or not self._collision_config.check_self:
            return []
        contacts = []
        for c in self._p.getContactPoints(self._robot_id, self._robot_id, physicsClientId=self._client):
            link_a = self._link_names.get(c[3], str(c[3]))
            link_b = self._link_names.get(c[4], str(c[4]))
            if should_ignore_contact(link_a, link_b, self._collision_config):
                continue
            contacts.append(
                CollisionContact(
                    backend="pybullet",
                    kind="self",
                    body_a="magician",
                    body_b="magician",
                    link_a=link_a,
                    link_b=link_b,
                    distance=float(c[8]),
                    normal_force=float(c[9]),
                )
            )
        return contacts

    def _update_debug_text(self, q_deg: list[float]) -> None:
        x, y, z, r = fk_magician(q_deg, tool=self._tool)
        suffix = ""
        if self._collision_stopped and self._last_collision is not None:
            suffix = f"\nCOLLISION STOP: {self._last_collision.link_a}/{self._last_collision.link_b}"
        text = f"EE  X={x:.1f}  Y={y:.1f}  Z={z:.1f}  R=0.0{suffix}"
        try:
            self._debug_text_id = self._p.addUserDebugText(
                text,
                [0.0, 0.0, 0.40],
                textColorRGB=[0.0, 0.0, 0.0],
                textSize=1.2,
                lifeTime=0,
                replaceItemUniqueId=self._debug_text_id,
                physicsClientId=self._client,
            )
        except Exception:
            pass

    def _write_joints(self) -> None:
        q_candidate = list(self._joint_deg)
        self._apply_visual_joints(q_candidate)
        contacts = self._collect_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._collision_stopped = True
            self._last_collision = contacts[0]
            self._joint_deg = list(self._last_safe_joint_deg)
            self._apply_visual_joints(self._joint_deg)
            print(f"\n[MagicianPyBullet] COLLISION STOP: {contact_summaries(contacts, limit=1)}")
        else:
            self._collision_stopped = False
            self._last_collision = None
            self._last_safe_joint_deg = q_candidate
        self._update_debug_text(self._joint_deg)

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

    def is_collision_stopped(self) -> bool:
        with self._lock:
            return self._collision_stopped

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
        try:
            self._p.disconnect(self._client)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

GUI_STEP_CHOICES = ("0.5", "1", "2", "5", "10")
DEFAULT_STEP_DEG = 2.0


class MagicianGui:
    def __init__(self, viewer: MagicianPyBulletViewer, step_deg: float) -> None:
        self._viewer = viewer
        self._root = tk.Tk()
        self._root.title("Magician PyBullet Teleop")
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
            text="Magician PyBullet Joint Teleop",
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
            lo, hi = MagicianPyBulletViewer.JOINT_LIMITS_DEG[joint_key]
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


def keyboard_loop(viewer: MagicianPyBulletViewer, step_deg: float) -> None:
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


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Magician PyBullet teleop")
    parser.add_argument(
        "--mode",
        choices=("gui", "keyboard"),
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
    viewer = MagicianPyBulletViewer(
        collision_guard=args.collision_guard,
        tool=args.ee_mode,
    )
    viewer.home()
    try:
        if args.mode == "keyboard":
            keyboard_loop(viewer, step_deg=args.step_deg)
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
