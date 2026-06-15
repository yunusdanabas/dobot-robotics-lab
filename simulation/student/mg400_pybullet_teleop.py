"""MG400 URDF teleop with PyBullet (Tk or keyboard). No ROS.

Run from dobot-robotics-lab package root. Needs pybullet. Keys listed in --help,
H, and argparse epilog.
"""

from __future__ import annotations

import argparse
import math
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
from simulation.runtime.kinematics import fk_mg400
from simulation.runtime.urdf_paths import resolve_mg400_urdf_path

# ---------------------------------------------------------------------------
# PyBullet viewer backend
# ---------------------------------------------------------------------------

class MG400PyBulletViewer:
    """Lightweight PyBullet wrapper that only visualises joint angles."""

    JOINT_BASE = "mg400_j1"
    JOINT_SHOULDER = "mg400_j2_1"
    JOINT_SHOULDER2 = "mg400_j2_2"
    JOINT_ELBOW = "mg400_j3_1"
    JOINT_ELBOW2 = "mg400_j3_2"
    JOINT_WRIST_P = "mg400_j4_1"
    JOINT_WRIST_P2 = "mg400_j4_2"
    JOINT_TOOL = "mg400_j5"
    EE_LINK = "mg400_end_effector_flange"
    BASE_LINK = "arm_frame_link_offset"
    SIMLAB_JOINTS = {
        "q1": "simlab_mg400_j1",
        "q2": "simlab_mg400_j2",
        "q3": "simlab_mg400_j3",
        "q4": "simlab_mg400_j4",
    }
    SIMLAB_TIP_LINK = "simlab_mg400_tip"

    JOINT_LIMITS_DEG = {
        "j1": (-160.0, 160.0),
        "j2": (-25.0, 85.0),
        "j3": (-25.0, 105.0),
        "j4": (-160.0, 160.0),
    }

    def __init__(self, collision_guard: bool | None = None):
        self._collision_config = CollisionGuardConfig.from_value(collision_guard)
        self._collision_stopped = False
        self._last_collision: CollisionContact | None = None
        _here = Path(__file__).resolve().parent
        _urdf = resolve_mg400_urdf_path()
        try:
            import pybullet as p
            import pybullet_data
        except ImportError as exc:
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
        self._link_idx_by_name = {name: idx for idx, name in self._link_names.items()}
        missing_simlab = [name for name in self.SIMLAB_JOINTS.values() if name not in self._joint_idx]
        if missing_simlab:
            raise RuntimeError(f"Prepared URDF missing simlab joints: {missing_simlab}")
        self._simlab_tip_link = self._joint_idx[self.SIMLAB_JOINTS["q4"]]
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

        # NOTE: No JOINT_GEAR constraints here.  The HarvestX URDF parallelogram
        # is maintained by setting the correct joint angles every frame (same
        # mapping as the RViz teleop).  Gear constraints cannot express the
        # non-constant relationships (e.g. j3_2 = -j2, j4_2 = +j3).

        self._lock = threading.Lock()
        self._joint_deg = [0.0, 0.0, 0.0, 0.0]  # body-frame q1,q2,q3,q4
        self._last_safe_joint_deg = list(self._joint_deg)
        self._ee_pose = fk_mg400(self._joint_deg)
        self._debug_text_id = -1
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _apply_visual_joints(self, q_deg: list[float]) -> None:
        """Apply joint angles using the same mapping as the RViz teleop.

        Body-frame controls (self._joint_deg) -> URDF joints:
            j1, j2, j3_body, j4  where j3_body = firmware_j3 - j2
        """
        p = self._p
        cid = self._client
        j1, j2, j3_body, j4 = q_deg
        j1_r = math.radians(j1)
        j2_r = math.radians(j2)
        j3_abs_r = math.radians(j2 + j3_body)   # firmware absolute elbow
        j4_r = math.radians(j4)
        mapping = {
            self.JOINT_BASE:      j1_r,
            self.JOINT_SHOULDER:  j2_r,
            self.JOINT_SHOULDER2: j2_r,
            self.JOINT_ELBOW:     math.radians(j3_body),  # j3 - j2
            self.JOINT_ELBOW2:    -j2_r,                   # -j2
            self.JOINT_WRIST_P:   -j3_abs_r,               # -j3
            self.JOINT_WRIST_P2:  j3_abs_r,                # +j3
            self.JOINT_TOOL:      j4_r,
            self.SIMLAB_JOINTS["q1"]: j1_r,
            self.SIMLAB_JOINTS["q2"]: j2_r,
            self.SIMLAB_JOINTS["q3"]: math.radians(j3_body),
            self.SIMLAB_JOINTS["q4"]: j4_r,
        }
        for name, angle in mapping.items():
            idx = self._joint_idx.get(name)
            if idx is not None:
                p.resetJointState(self._robot_id, idx, angle, physicsClientId=cid)
        p.stepSimulation(physicsClientId=cid)

    def _query_sim_ee_pose_unlocked(self, q_deg: list[float]) -> tuple[float, float, float, float]:
        """Return API-frame simlab tip pose from PyBullet in mm/deg.

        Called from the simulation update thread while ``self._lock`` is held.
        """
        try:
            state = self._p.getLinkState(
                self._robot_id,
                self._simlab_tip_link,
                computeForwardKinematics=True,
                physicsClientId=self._client,
            )
            x, y, z = state[0]
            r = math.degrees(
                self._p.getJointState(
                    self._robot_id,
                    self._joint_idx[self.SIMLAB_JOINTS["q4"]],
                    physicsClientId=self._client,
                )[0]
            )
            return (float(x) * 1000.0, float(y) * 1000.0, float(z) * 1000.0, float(r))
        except Exception:
            return fk_mg400(q_deg)

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
                    body_a="mg400",
                    body_b="mg400",
                    link_a=link_a,
                    link_b=link_b,
                    distance=float(c[8]),
                    normal_force=float(c[9]),
                )
            )
        return contacts

    def _update_debug_text(self, pose: tuple[float, float, float, float]) -> None:
        x, y, z, r = pose
        suffix = ""
        if self._collision_stopped and self._last_collision is not None:
            suffix = f"\nCOLLISION STOP: {self._last_collision.link_a}/{self._last_collision.link_b}"
        text = f"API EE  X={x:.1f}  Y={y:.1f}  Z={z:.1f}  R={r:.1f}{suffix}"
        try:
            self._debug_text_id = self._p.addUserDebugText(
                text,
                [0.0, 0.0, 0.45],
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
        candidate_pose = self._query_sim_ee_pose_unlocked(q_candidate)
        contacts = self._collect_collision_contacts()
        if contacts and self._collision_config.stop_on_collision:
            self._collision_stopped = True
            self._last_collision = contacts[0]
            self._joint_deg = list(self._last_safe_joint_deg)
            self._apply_visual_joints(self._joint_deg)
            self._ee_pose = self._query_sim_ee_pose_unlocked(self._joint_deg)
            print(f"\n[MG400PyBullet] COLLISION STOP: {contact_summaries(contacts, limit=1)}")
        else:
            self._collision_stopped = False
            self._last_collision = None
            self._last_safe_joint_deg = q_candidate
            self._ee_pose = candidate_pose
        self._update_debug_text(self._ee_pose)

    def _update_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                self._write_joints()
            time.sleep(1.0 / 20.0)

    def center(self) -> None:
        with self._lock:
            self._joint_deg = [0.0, 0.0, 0.0, 0.0]

    def randomize(self) -> None:
        with self._lock:
            self._joint_deg = [
                random.uniform(*self.JOINT_LIMITS_DEG["j1"]),
                random.uniform(*self.JOINT_LIMITS_DEG["j2"]),
                random.uniform(*self.JOINT_LIMITS_DEG["j3"]),
                random.uniform(*self.JOINT_LIMITS_DEG["j4"]),
            ]

    def get_joint_deg(self) -> list[float]:
        with self._lock:
            return list(self._joint_deg)

    def get_ee_pose(self) -> tuple[float, float, float, float]:
        with self._lock:
            return tuple(self._ee_pose)

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
        joint_name = f"j{index + 1}"
        lo, hi = self.JOINT_LIMITS_DEG[joint_name]
        with self._lock:
            self._joint_deg[index] = max(lo, min(hi, float(value_deg)))

    def nudge_deg(self, index: int, delta_deg: float) -> None:
        with self._lock:
            joint_name = f"j{index + 1}"
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


class MG400Gui:
    def __init__(self, viewer: MG400PyBulletViewer, step_deg: float) -> None:
        self._viewer = viewer
        self._root = tk.Tk()
        self._root.title("MG400 PyBullet Teleop")
        self._root.geometry("760x440")
        self._root.minsize(760, 400)
        self._root.bind("<Escape>", lambda _event: self._root.destroy())
        self._root.bind("<c>", lambda _event: self._center())
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
            text="MG400 PyBullet Joint Teleop",
            font=("TkDefaultFont", 13, "bold"),
        )
        header.pack(anchor="w")

        helper = ttk.Label(
            outer,
            text="Sliders are in degrees. Buttons apply the current step size.",
        )
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
        ttk.Button(controls, text="Center", command=self._center).pack(side="left", padx=(0, 8))
        ttk.Button(controls, text="Random", command=self._randomize).pack(side="left")

        for idx, joint_name in enumerate(("J1", "J2", "J3", "J4")):
            row = ttk.Frame(outer)
            row.pack(fill="x", pady=6)

            ttk.Label(row, text=joint_name, width=4).pack(side="left")
            ttk.Button(row, text="-", width=4, command=lambda i=idx: self._nudge(i, -1.0)).pack(side="left")

            joint_key = f"j{idx + 1}"
            lo, hi = MG400PyBulletViewer.JOINT_LIMITS_DEG[joint_key]
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
            text="Esc closes the window. C centers. R picks a random valid pose.",
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

    def _center(self) -> None:
        self._viewer.center()
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
        self._root.after(100, self._poll_status_labels)
        self._root.mainloop()

    def _poll_status_labels(self) -> None:
        try:
            self._sync_labels()
            self._root.after(100, self._poll_status_labels)
        except tk.TclError:
            pass


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
    "k": (3, -1.0),
    "i": (3, +1.0),
}


def keyboard_loop(viewer: MG400PyBulletViewer, step_deg: float) -> None:
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
        j1, j2, j3, j4 = viewer.get_joint_deg()
        x, y, z, r = viewer.get_ee_pose()
        print(
            f"  J1={j1:6.1f} J2={j2:6.1f} J3={j3:6.1f} J4={j4:6.1f} "
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
                viewer.center()
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
    parser = argparse.ArgumentParser(description="MG400 PyBullet teleop")
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
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    viewer = MG400PyBulletViewer(collision_guard=args.collision_guard)
    viewer.center()
    try:
        if args.mode == "keyboard":
            keyboard_loop(viewer, step_deg=args.step_deg)
        else:
            try:
                MG400Gui(viewer, step_deg=args.step_deg).run()
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
