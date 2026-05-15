#!/usr/bin/env python3
"""
robot_parity_diagnostic.py - Real/simulation parity diagnostics.

This script sends the same small, safe command set to either a simulation
backend or a real robot, records detailed feedback as JSON Lines, and can
compare two runs afterward.

No hardware motion happens unless all of these are true:
  - subcommand is ``run``
  - ``--target real`` is selected
  - ``--confirm-real`` is supplied
  - ``--dry-run`` is not supplied
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import re
import sys
import threading
import time
import traceback
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
SIM_RUNTIME = ROOT / "simulation" / "runtime"
MAGICIAN_UTILS = ROOT / "robots" / "magician" / "utils.py"
MG400_UTILS = ROOT / "robots" / "mg400" / "utils_mg400.py"
DEFAULT_RESULTS_DIR = ROOT / "simulation" / "diagnostics" / "results"

if str(SIM_RUNTIME) not in sys.path:
    sys.path.insert(0, str(SIM_RUNTIME))

from kinematics import angle_error_deg, fk_magician, fk_mg400_api, max_position_error  # noqa: E402


MAGICIAN_BACKENDS = ("pybullet", "mujoco")
MG400_BACKENDS = ("pybullet", "mujoco")
CORE_SUITES = ("joint", "cartesian")
EXTRA_SUITES = (
    "axis", "repeatability", "equivalence", "modes", "workspace", "io", "connection",
    "pick_place", "trajectory", "speed_factor", "feed_parity",
)
SUITE_CHOICES = CORE_SUITES + EXTRA_SUITES + ("all",)
REPEATABILITY_CYCLES = 3
REFERENCE_POSES = {
    "magician": (200.0, 0.0, 100.0, 0.0),
    "mg400": (300.0, 0.0, 50.0, 0.0),
}


@dataclass(frozen=True)
class DiagnosticCase:
    robot: str
    suite: str
    name: str
    command: str
    values: tuple | None  # None for multi-step commands (sequence, speed_sweep, feed_sample)
    note: str = ""
    steps: tuple | None = None   # for "sequence": tuple of (cmd_type, values) pairs
    speeds: tuple | None = None  # for "speed_sweep": tuple of SpeedFactor ints
    feed_samples: int = 0        # for "feed_sample": number of feed-vs-dashboard samples


class AngleMode:
    """MODE_PTP.MOVJ_ANGLE-compatible fallback for simulation imports."""

    name = "MOVJ_ANGLE"

    def __str__(self) -> str:
        return self.name


CASES: dict[str, dict[str, list[DiagnosticCase]]] = {
    "magician": {
        "joint": [
            DiagnosticCase("magician", "joint", "joint_zero", "joint", (0.0, 0.0, 0.0, 0.0), "Joint-space zero/home."),
            DiagnosticCase("magician", "joint", "base_left_small", "joint", (15.0, 0.0, 10.0, 0.0), "Small positive base rotation."),
            DiagnosticCase("magician", "joint", "base_right_small", "joint", (-15.0, 0.0, 10.0, 0.0), "Small negative base rotation."),
            DiagnosticCase("magician", "joint", "elbow_lift_small", "joint", (0.0, 5.0, 5.0, 15.0), "Small shoulder/elbow lift."),
        ],
        "cartesian": [
            DiagnosticCase("magician", "cartesian", "cart_ready", "cartesian", (200.0, 0.0, 100.0, 0.0), "Ready pose."),
            DiagnosticCase("magician", "cartesian", "cart_left_small", "cartesian", (220.0, 40.0, 110.0, 15.0), "Small +Y Cartesian move."),
            DiagnosticCase("magician", "cartesian", "cart_right_small", "cartesian", (190.0, -40.0, 105.0, -15.0), "Small -Y Cartesian move."),
        ],
        "axis": [
            DiagnosticCase("magician", "axis", "axis_y_plus", "relative", (0.0, 20.0, 0.0, 0.0), "Small +Y move from current pose."),
            DiagnosticCase("magician", "axis", "axis_y_minus", "relative", (0.0, -20.0, 0.0, 0.0), "Small -Y move back."),
            DiagnosticCase("magician", "axis", "axis_z_plus", "relative", (0.0, 0.0, 15.0, 0.0), "Small +Z lift."),
            DiagnosticCase("magician", "axis", "axis_z_minus", "relative", (0.0, 0.0, -15.0, 0.0), "Small -Z return."),
            DiagnosticCase("magician", "axis", "axis_r_plus", "relative", (0.0, 0.0, 0.0, 15.0), "Small +R rotation."),
            DiagnosticCase("magician", "axis", "axis_r_minus", "relative", (0.0, 0.0, 0.0, -15.0), "Small -R return."),
        ],
        "repeatability": [
            DiagnosticCase("magician", "repeatability", "repeat_joint_small", "repeat_joint", (10.0, 0.0, 10.0, 10.0), "Repeat home -> target several times."),
        ],
        "equivalence": [
            DiagnosticCase("magician", "equivalence", "joint_vs_cart_small", "equivalence", (10.0, 0.0, 10.0, 10.0), "Joint target vs same expected Cartesian target."),
        ],
        "modes": [
            DiagnosticCase("magician", "modes", "cart_mode_compare", "mode_compare", (220.0, 20.0, 110.0, 10.0), "Compare default Cartesian move against linear fallback when available."),
        ],
        "workspace": [],
        "io": [
            DiagnosticCase("magician", "io", "tool_on", "io", (1.0, 1.0, 0.0, 0.0), "Turn suction output on without payload."),
            DiagnosticCase("magician", "io", "tool_off", "io", (1.0, 0.0, 0.0, 0.0), "Turn suction output off."),
        ],
        "connection": [
            DiagnosticCase("magician", "connection", "snapshot_1", "snapshot", (0.0, 0.0, 0.0, 0.0), "No-motion feedback snapshot."),
            DiagnosticCase("magician", "connection", "snapshot_2", "snapshot", (0.0, 0.0, 0.0, 0.0), "Second no-motion feedback snapshot for stale feedback checks."),
        ],
    },
    "mg400": {
        "joint": [
            DiagnosticCase("mg400", "joint", "joint_zero", "joint", (0.0, 0.0, 0.0, 0.0), "Joint-space zero/home."),
            DiagnosticCase("mg400", "joint", "base_left_small", "joint", (15.0, -10.0, 20.0, 15.0), "Small positive base rotation."),
            DiagnosticCase("mg400", "joint", "base_right_small", "joint", (-15.0, -10.0, 20.0, -15.0), "Small negative base rotation."),
            DiagnosticCase("mg400", "joint", "elbow_lift_small", "joint", (0.0, -15.0, 30.0, 0.0), "Small elbow motion, Z stays high."),
        ],
        "cartesian": [
            DiagnosticCase("mg400", "cartesian", "cart_ready", "cartesian", (300.0, 0.0, 50.0, 0.0), "Ready pose."),
            DiagnosticCase("mg400", "cartesian", "cart_left_small", "cartesian", (320.0, 60.0, 80.0, 15.0), "Small +Y Cartesian move."),
            DiagnosticCase("mg400", "cartesian", "cart_right_small", "cartesian", (260.0, -60.0, 90.0, -15.0), "Small -Y Cartesian move."),
        ],
        "axis": [
            DiagnosticCase("mg400", "axis", "axis_y_plus", "relative", (0.0, 30.0, 0.0, 0.0), "Small +Y move from current pose."),
            DiagnosticCase("mg400", "axis", "axis_y_minus", "relative", (0.0, -30.0, 0.0, 0.0), "Small -Y move back."),
            DiagnosticCase("mg400", "axis", "axis_z_plus", "relative", (0.0, 0.0, 15.0, 0.0), "Small +Z lift."),
            DiagnosticCase("mg400", "axis", "axis_z_minus", "relative", (0.0, 0.0, -15.0, 0.0), "Small -Z return; stays non-negative."),
            DiagnosticCase("mg400", "axis", "axis_r_plus", "relative", (0.0, 0.0, 0.0, 15.0), "Small +R rotation."),
            DiagnosticCase("mg400", "axis", "axis_r_minus", "relative", (0.0, 0.0, 0.0, -15.0), "Small -R return."),
        ],
        "repeatability": [
            DiagnosticCase("mg400", "repeatability", "repeat_joint_small", "repeat_joint", (10.0, -10.0, 20.0, 10.0), "Repeat home -> target several times."),
        ],
        "equivalence": [
            DiagnosticCase("mg400", "equivalence", "joint_vs_cart_small", "equivalence", (10.0, -10.0, 20.0, 10.0), "Joint target vs same expected Cartesian target."),
        ],
        "modes": [
            DiagnosticCase("mg400", "modes", "movj_vs_movl_small", "mode_compare", (320.0, 40.0, 80.0, 10.0), "Compare MovJ and MovL final pose."),
        ],
        "workspace": [],
        "io": [
            DiagnosticCase("mg400", "io", "tool_on", "io", (1.0, 1.0, 0.0, 0.0), "Turn ToolDO output on without payload."),
            DiagnosticCase("mg400", "io", "tool_off", "io", (1.0, 0.0, 0.0, 0.0), "Turn ToolDO output off."),
        ],
        "connection": [
            DiagnosticCase("mg400", "connection", "snapshot_1", "snapshot", (0.0, 0.0, 0.0, 0.0), "No-motion feedback snapshot."),
            DiagnosticCase("mg400", "connection", "snapshot_2", "snapshot", (0.0, 0.0, 0.0, 0.0), "Second no-motion feedback snapshot for stale feedback checks."),
        ],
        "pick_place": [
            DiagnosticCase(
                "mg400", "pick_place", "pick_place_01", "sequence", None,
                note="Full pick-and-place: approach, descend, suction ON/OFF, lift, return home.",
                steps=(
                    ("move_cartesian", (280.0, -80.0, 70.0, 0.0)),
                    ("move_cartesian", (280.0, -80.0, 20.0, 0.0)),
                    ("io",             (1, 1)),
                    ("move_cartesian", (280.0, -80.0, 70.0, 0.0)),
                    ("move_cartesian", (280.0,  80.0, 70.0, 0.0)),
                    ("move_cartesian", (280.0,  80.0, 20.0, 0.0)),
                    ("io",             (1, 0)),
                    ("move_cartesian", (280.0,  80.0, 70.0, 0.0)),
                    ("move_cartesian", (300.0,   0.0, 50.0, 0.0)),
                ),
            ),
        ],
        "trajectory": [
            DiagnosticCase(
                "mg400", "trajectory", "traj_xy_sweep", "sequence", None,
                note="5-waypoint XY sweep at fixed Z=80 mm.",
                steps=(
                    ("move_cartesian", (300.0,    0.0, 80.0, 0.0)),
                    ("move_cartesian", (250.0, -100.0, 80.0, 0.0)),
                    ("move_cartesian", (230.0,    0.0, 80.0, 0.0)),
                    ("move_cartesian", (250.0,  100.0, 80.0, 0.0)),
                    ("move_cartesian", (300.0,    0.0, 80.0, 0.0)),
                ),
            ),
            DiagnosticCase(
                "mg400", "trajectory", "traj_z_sweep", "sequence", None,
                note="Z-axis lift-and-lower at fixed XY=280,0.",
                steps=(
                    ("move_cartesian", (280.0, 0.0,  50.0, 0.0)),
                    ("move_cartesian", (280.0, 0.0, 100.0, 0.0)),
                    ("move_cartesian", (280.0, 0.0,  50.0, 0.0)),
                ),
            ),
        ],
        "speed_factor": [
            DiagnosticCase(
                "mg400", "speed_factor", "speed_10", "speed_sweep", (280.0, 0.0, 80.0, 0.0),
                note="Move to target at SpeedFactor=10; measure final pose.", speeds=(10,),
            ),
            DiagnosticCase(
                "mg400", "speed_factor", "speed_30", "speed_sweep", (280.0, 0.0, 80.0, 0.0),
                note="Move to target at SpeedFactor=30; measure final pose.", speeds=(30,),
            ),
            DiagnosticCase(
                "mg400", "speed_factor", "speed_70", "speed_sweep", (280.0, 0.0, 80.0, 0.0),
                note="Move to target at SpeedFactor=70; measure final pose.", speeds=(70,),
            ),
        ],
        "feed_parity": [
            DiagnosticCase(
                "mg400", "feed_parity", "feed_static", "feed_sample", (300.0, 0.0, 50.0, 0.0),
                note="Static READY_POSE: compare port-30004 feed vs GetPose() dashboard.", feed_samples=20,
            ),
            DiagnosticCase(
                "mg400", "feed_parity", "feed_post_move", "feed_sample", (280.0, 60.0, 80.0, 0.0),
                note="After move: compare feed vs dashboard at 50 ms intervals.", feed_samples=20,
            ),
        ],
    },
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def run_id(robot: str, target: str) -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{robot}_{target}_{stamp}"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load module {name!r} from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def as_float_tuple(values: Any, length: int = 4) -> tuple[float, ...]:
    if values is None:
        raise ValueError("Expected values, got None")
    result = tuple(float(v) for v in values)
    if len(result) != length:
        raise ValueError(f"Expected {length} values, got {len(result)}: {values!r}")
    return result


def vector_error(expected: tuple[float, float, float, float] | None, actual: tuple[float, float, float, float] | None) -> dict[str, float] | None:
    if expected is None or actual is None:
        return None
    dx = float(actual[0] - expected[0])
    dy = float(actual[1] - expected[1])
    dz = float(actual[2] - expected[2])
    dr = angle_error_deg(expected[3], actual[3])
    return {
        "dx_mm": dx,
        "dy_mm": dy,
        "dz_mm": dz,
        "max_abs_xyz_mm": max(abs(dx), abs(dy), abs(dz)),
        "euclidean_xyz_mm": math.sqrt(dx * dx + dy * dy + dz * dz),
        "rotation_deg": dr,
    }


def pose_delta(before: tuple[float, float, float, float] | None, after: tuple[float, float, float, float] | None) -> dict[str, float] | None:
    if before is None or after is None:
        return None
    return {
        "dx_mm": after[0] - before[0],
        "dy_mm": after[1] - before[1],
        "dz_mm": after[2] - before[2],
        "dr_deg": after[3] - before[3],
    }


def sample_stats(samples: list[dict[str, Any]]) -> dict[str, float] | None:
    poses = [s.get("pose") for s in samples if s.get("pose") is not None]
    if len(poses) < 2:
        return None
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    zs = [p[2] for p in poses]
    rs = [p[3] for p in poses]
    return {
        "span_x_mm": max(xs) - min(xs),
        "span_y_mm": max(ys) - min(ys),
        "span_z_mm": max(zs) - min(zs),
        "span_r_deg": max(rs) - min(rs),
        "max_xyz_span_mm": max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)),
    }


def expected_pose(case: DiagnosticCase) -> tuple[float, float, float, float] | None:
    if case.command in ("sequence", "speed_sweep", "feed_sample"):
        return None  # multi-step commands have no single expected pose
    if case.command == "cartesian":
        return case.values
    if case.command in ("relative", "snapshot", "io"):
        return None
    if case.command in ("repeat_joint", "equivalence"):
        if case.robot == "magician":
            return fk_magician(case.values)
        if case.robot == "mg400":
            return fk_mg400_api(case.values)
    if case.command == "mode_compare":
        return case.values
    if case.robot == "magician":
        return fk_magician(case.values)
    if case.robot == "mg400":
        return fk_mg400_api(case.values)
    raise ValueError(f"Unknown robot: {case.robot}")


def mg400_body_to_firmware(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    q1, q2, q3, q4 = q
    return q1, q2, q2 + q3, q4


def mg400_firmware_to_body(j: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    j1, j2, j3, j4 = j
    return j1, j2, j3 - j2, j4


def numbers_from_response(response: Any) -> list[float]:
    return [float(n) for n in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(response))]


def _is_empty_mg400_response(response: Any) -> bool:
    if response is None:
        return True
    if isinstance(response, (bytes, bytearray)):
        return len(bytes(response).strip()) == 0
    return not str(response).strip()


def require_mg400_response(response: Any, command: str) -> Any:
    """Raise a clear connection error when the vendor SDK returns no payload."""
    if _is_empty_mg400_response(response):
        raise ConnectionError(
            f"Empty {command} response from MG400. The TCP socket is closed or "
            "desynchronized; disconnect/reconnect the GUI and wait for the "
            "controller to release stale sockets."
        )
    return response


def require_mg400_success(response: Any, command: str) -> Any:
    response = require_mg400_response(response, command)
    nums = numbers_from_response(response)
    if nums and int(nums[0]) != 0:
        raise RuntimeError(f"{command} returned status {int(nums[0])}: {response!r}")
    return response


def parse_mg400_pose(response: Any) -> tuple[float, float, float, float]:
    response = require_mg400_response(response, "GetPose")
    nums = numbers_from_response(response)
    if len(nums) >= 5:
        status = int(nums[0])
        if status != 0:
            raise ValueError(f"GetPose returned status {status}: {response!r}")
        return nums[1], nums[2], nums[3], nums[4]
    if len(nums) == 4:
        return nums[0], nums[1], nums[2], nums[3]
    raise ValueError(f"Cannot parse MG400 pose from {response!r}")


def parse_mg400_angles(response: Any) -> tuple[float, float, float, float]:
    response = require_mg400_response(response, "GetAngle")
    nums = numbers_from_response(response)
    if len(nums) >= 5:
        status = int(nums[0])
        if status != 0:
            raise ValueError(f"GetAngle returned status {status}: {response!r}")
        return nums[1], nums[2], nums[3], nums[4]
    if len(nums) == 4:
        return nums[0], nums[1], nums[2], nums[3]
    raise ValueError(f"Cannot parse MG400 angles from {response!r}")


def parse_mg400_mode(response: Any) -> int | None:
    response = require_mg400_response(response, "RobotMode")
    nums = [int(n) for n in re.findall(r"[-+]?\d+", str(response))]
    if not nums:
        return None
    if len(nums) >= 2 and nums[0] in (0, -1):
        if nums[0] != 0:
            raise ValueError(f"RobotMode returned status {nums[0]}: {response!r}")
        return nums[1]
    return nums[-1]


def parse_mg400_error_ids(response: Any) -> list[int] | None:
    response = require_mg400_response(response, "GetErrorID")
    text = str(response or "")
    if "[" not in text or "]" not in text:
        return None
    nums = [int(n) for n in re.findall(r"[-+]?\d+", text)]
    if not nums:
        return None
    if nums[0] != 0:
        raise ValueError(f"GetErrorID returned status {nums[0]}: {response!r}")
    return [n for n in nums[1:] if n != 0]


def mag_pose_to_tuple(pose: Any) -> tuple[float, float, float, float]:
    if hasattr(pose, "position"):
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.position.r),
        )
    if isinstance(pose, (tuple, list)) and len(pose) >= 4:
        return float(pose[0]), float(pose[1]), float(pose[2]), float(pose[3])
    raise ValueError(f"Unsupported Magician pose format: {type(pose)!r}")


def mag_joints_to_tuple(pose: Any) -> tuple[float, float, float, float]:
    if hasattr(pose, "joints"):
        return (
            float(pose.joints.j1),
            float(pose.joints.j2),
            float(pose.joints.j3),
            float(pose.joints.j4),
        )
    if isinstance(pose, (tuple, list)) and len(pose) >= 8:
        return float(pose[4]), float(pose[5]), float(pose[6]), float(pose[7])
    raise ValueError(f"Unsupported Magician joints format: {type(pose)!r}")


def alarm_names(alarms: Any) -> list[str]:
    result = []
    for alarm in alarms or []:
        result.append(str(getattr(alarm, "name", alarm)))
    return result


class JsonlWriter:
    def __init__(self, path: Path):
        self.path = path
        self._fh = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", encoding="utf-8")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fh is not None:
            self._fh.close()

    def write(self, record: dict[str, Any]) -> None:
        if self._fh is None:
            raise RuntimeError("JsonlWriter is not open")
        self._fh.write(json.dumps(record, sort_keys=True) + "\n")
        self._fh.flush()


class BaseRobotClient:
    def connect(self) -> None:
        raise NotImplementedError

    def snapshot(self) -> dict[str, Any]:
        raise NotImplementedError

    def move_joint(self, q: tuple[float, float, float, float]) -> Any:
        raise NotImplementedError

    def move_cartesian(self, pose: tuple[float, float, float, float]) -> Any:
        raise NotImplementedError

    def move_cartesian_linear(self, pose: tuple[float, float, float, float]) -> Any:
        return self.move_cartesian(pose)

    def set_tool(self, index: int, state: bool) -> Any:
        raise NotImplementedError

    def set_speed(self, factor: int) -> None:
        pass  # default no-op for robots that don't support SpeedFactor

    def start_feed(self) -> None:
        pass  # start port-30004 feedback thread if applicable

    def read_feed_pose(self) -> tuple[float, float, float, float] | None:
        return None  # returns (x, y, z, r) from live feed, or None if unavailable

    def close(self) -> None:
        raise NotImplementedError



class MagicianSimClient(BaseRobotClient):
    def __init__(self, backend: str, gui: bool, ee_mode: str):
        self.backend = backend
        self.gui = gui
        self.ee_mode = ee_mode
        self.bot = None

    def connect(self) -> None:
        from sim_dobot import SimDobot

        self.bot = SimDobot(backend=self.backend, gui=self.gui, tool=self.ee_mode)

    def snapshot(self) -> dict[str, Any]:
        assert self.bot is not None
        pose_obj = self.bot.get_pose()
        raw: dict[str, Any] = {"backend": self.backend, "target": "sim"}
        if hasattr(self.bot, "collision_summary"):
            raw["collision_summary"] = self.bot.collision_summary()
            raw["collision_stopped"] = bool(self.bot.is_collision_stopped())
        return {"pose": mag_pose_to_tuple(pose_obj), "joints": mag_joints_to_tuple(pose_obj), "raw": raw}

    def move_joint(self, q: tuple[float, float, float, float]) -> Any:
        assert self.bot is not None
        return self.bot.move_to(*q, wait=True, mode=AngleMode())

    def move_cartesian(self, pose: tuple[float, float, float, float]) -> Any:
        assert self.bot is not None
        return self.bot.move_to(*pose, wait=True)

    def set_tool(self, index: int, state: bool) -> Any:
        assert self.bot is not None
        self.bot.suck(bool(state))
        return {"suck": bool(state), "index": index}

    def close(self) -> None:
        if self.bot is not None:
            try:
                self.bot.move_to(0, 0, 0, 0, wait=True, mode=AngleMode())
            except Exception:
                pass
            self.bot.close()
            self.bot = None


class MagicianRealClient(BaseRobotClient):
    def __init__(self, port: str | None, return_home: bool):
        self.port = port
        self.return_home = return_home
        self.bot = None
        self.utils = None
        self.mode_ptp = None
        self.connected_port = None

    def connect(self) -> None:
        self.utils = load_module("diagnostic_magician_utils", MAGICIAN_UTILS)
        from pydobotplus import Dobot
        from pydobotplus.dobotplus import MODE_PTP

        self.mode_ptp = MODE_PTP
        self.connected_port = self.port or self.utils.find_port()
        if self.connected_port is None:
            raise OSError(
                "No Dobot Magician serial port found. Set DOBOT_PORT or pass --port."
            )
        self.bot = Dobot(port=self.connected_port)
        self.utils.prepare_robot(self.bot)
        self._raise_if_alarm_active("connect")

    def _raise_if_alarm_active(self, context: str) -> None:
        assert self.bot is not None
        try:
            alarms = self.bot.get_alarms()
        except Exception as exc:
            raise RuntimeError(f"Unable to read Magician alarms after {context}: {exc}") from exc
        if not alarms:
            return
        names = alarm_names(alarms)
        try:
            self.bot.clear_alarms()
        except Exception:
            pass
        raise RuntimeError(
            f"Magician reported active alarms after {context}: {names}. "
            "Alarms were cleared; rerun from a safe pose."
        )

    def snapshot(self) -> dict[str, Any]:
        assert self.bot is not None
        pose_obj = self.bot.get_pose()
        alarms = []
        try:
            alarms = alarm_names(self.bot.get_alarms())
        except Exception as exc:
            alarms = [f"alarm_read_failed: {exc}"]
        return {
            "pose": mag_pose_to_tuple(pose_obj),
            "joints": mag_joints_to_tuple(pose_obj),
            "raw": {"target": "real", "port": self.connected_port, "alarms": alarms},
        }

    def move_joint(self, q: tuple[float, float, float, float]) -> Any:
        assert self.bot is not None and self.mode_ptp is not None
        response = self.bot.move_to(*q, wait=True, mode=self.mode_ptp.MOVJ_ANGLE)
        self._raise_if_alarm_active("joint move")
        return response

    def move_cartesian(self, pose: tuple[float, float, float, float]) -> Any:
        assert self.bot is not None and self.utils is not None
        response = self.utils.safe_move(self.bot, *pose, verify=True)
        self._raise_if_alarm_active("cartesian move")
        return response

    def set_tool(self, index: int, state: bool) -> Any:
        assert self.bot is not None
        self.bot.suck(bool(state))
        self._raise_if_alarm_active("tool command")
        return {"suck": bool(state), "index": index}

    def close(self) -> None:
        if self.bot is None:
            return
        try:
            if self.return_home and self.mode_ptp is not None:
                self.bot.move_to(0, 0, 0, 0, wait=True, mode=self.mode_ptp.MOVJ_ANGLE)
        except Exception:
            pass
        try:
            self.bot.close()
        finally:
            self.bot = None


class MG400SimClient(BaseRobotClient):
    def __init__(self, backend: str, gui: bool):
        self.backend = backend
        self.gui = gui
        self.sim = None
        self.dashboard = None
        self.move_api = None
        self.feed = None
        self._lock = threading.Lock()           # guards all pybullet/mujoco calls
        self._mj_renderer = None               # cached mujoco.Renderer (fix2)
        self._mj_renderer_size: tuple = (0, 0)

    def connect(self) -> None:
        from sim_mg400 import SimMG400

        self.sim = SimMG400(backend=self.backend, gui=self.gui)
        self.dashboard, self.move_api, self.feed = self.sim.connect()
        self.dashboard.EnableRobot()

    def snapshot(self) -> dict[str, Any]:
        assert self.dashboard is not None
        with self._lock:
            pose_raw = self.dashboard.GetPose()
            angle_raw = self.dashboard.GetAngle()
            mode_raw = self.dashboard.RobotMode()
            error_raw = self.dashboard.GetErrorID()
        pose = parse_mg400_pose(pose_raw)
        firmware_angles = parse_mg400_angles(angle_raw)
        raw = {
            "target": "sim",
            "backend": self.backend,
            "GetPose": str(pose_raw),
            "GetAngle": str(angle_raw),
            "RobotMode": str(mode_raw),
            "GetErrorID": str(error_raw),
            "mode": parse_mg400_mode(mode_raw),
            "error_ids": parse_mg400_error_ids(error_raw),
        }
        if self.sim is not None and hasattr(self.sim, "collision_summary"):
            raw["collision_summary"] = self.sim.collision_summary()
            raw["collision_stopped"] = bool(self.sim.is_collision_stopped())
        return {"pose": pose, "joints": mg400_firmware_to_body(firmware_angles), "raw": raw}

    def move_joint(self, q: tuple[float, float, float, float]) -> Any:
        assert self.sim is not None
        firmware = mg400_body_to_firmware(q)
        self.sim.interpolate_joints_to(
            list(mg400_firmware_to_body(firmware)), steps=20, pause=0.016, lock=self._lock
        )
        return {"JointMovJ": "interpolated", "Sync": "ok", "firmware_joints": firmware}

    def move_cartesian(self, pose: tuple[float, float, float, float]) -> Any:
        assert self.sim is not None
        x, y, z, r = pose
        z = max(30.0, z)   # sim Z floor: self-collision below ~30mm
        self.sim.interpolate_to(x, y, z, r, steps=20, pause=0.016, lock=self._lock)
        return {"MovJ": "interpolated", "Sync": "ok"}

    def move_cartesian_linear(self, pose: tuple[float, float, float, float]) -> Any:
        assert self.sim is not None
        x, y, z, r = pose
        z = max(30.0, z)   # sim Z floor: self-collision below ~30mm
        self.sim.interpolate_to(x, y, z, r, steps=20, pause=0.016, lock=self._lock)
        return {"MovL": "interpolated", "Sync": "ok"}

    def set_tool(self, index: int, state: bool) -> Any:
        assert self.move_api is not None
        with self._lock:
            result = self.move_api.ToolDO(index, int(bool(state)))
        return {"ToolDO": result, "index": index, "state": bool(state)}

    def set_speed(self, factor: int) -> None:
        if self.dashboard is not None:
            with self._lock:
                self.dashboard.SpeedFactor(factor)

    def read_feed_pose(self) -> tuple[float, float, float, float] | None:
        if self.feed is None:
            return None
        try:
            cp = self.feed.current_pose
            return (float(cp["x"]), float(cp["y"]), float(cp["z"]), float(cp["r"]))
        except Exception:
            return None

    def get_camera_frame(self, width: int = 480, height: int = 360,
                         yaw: float = 45, pitch: float = -45,
                         distance: float = 0.8) -> bytes | None:
        if self.sim is None:
            return None
        impl = self.sim._impl
        # PyBullet backend — lock guards concurrent pybullet C-extension calls
        if hasattr(impl, "_p") and hasattr(impl, "_client"):
            import numpy as np
            with self._lock:
                p, cid = impl._p, impl._client
                # Dynamic target: follow the robot EE, fall back to workspace centre
                try:
                    ee_state = p.getLinkState(impl._robot_id, impl._ee_link,
                                              computeForwardKinematics=True,
                                              physicsClientId=cid)
                    cam_target = list(ee_state[0])
                except Exception:
                    cam_target = [0.28, 0.0, 0.10]
                view = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=cam_target,
                    distance=distance, yaw=yaw, pitch=pitch,
                    roll=0, upAxisIndex=2,
                )
                proj = p.computeProjectionMatrixFOV(
                    fov=60, aspect=width / height,
                    nearVal=0.05, farVal=3.0,   # nearVal 0.01→0.05 prevents plane clipping
                    physicsClientId=cid,
                )
                _, _, rgb, _, _ = p.getCameraImage(width, height, view, proj,
                                                    physicsClientId=cid)
            return np.array(rgb, dtype=np.uint8)[:, :, :3].tobytes()
        # MuJoCo backend — cache Renderer to avoid per-frame allocation
        if hasattr(impl, "_model") and hasattr(impl, "_data"):
            import os
            import mujoco
            import numpy as np
            os.environ["MUJOCO_GL"] = os.environ.get("MUJOCO_GL", "egl")
            with self._lock:
                if impl._model is None or impl._data is None:
                    return None
                if self._mj_renderer is None or self._mj_renderer_size != (width, height):
                    if self._mj_renderer is not None:
                        try:
                            self._mj_renderer.close()
                        except Exception:
                            pass
                    self._mj_renderer = mujoco.Renderer(impl._model, height, width)
                    self._mj_renderer_size = (width, height)
                mujoco.mj_forward(impl._model, impl._data)
                cam = mujoco.MjvCamera()
                cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
                cam.lookat[:] = [0.28, 0.0, 0.10]
                cam.azimuth   = yaw
                cam.elevation = pitch
                cam.distance  = distance
                self._mj_renderer.update_scene(impl._data, cam)
                rgb = self._mj_renderer.render()
            arr = np.array(rgb, dtype=np.uint8)
            if arr.ndim == 3 and arr.shape[2] == 4:
                arr = arr[:, :, :3]   # strip alpha if RGBA (future MuJoCo versions)
            return arr.tobytes()
        return None

    def close(self) -> None:
        if self._mj_renderer is not None:
            try:
                self._mj_renderer.close()
            except Exception:
                pass
            self._mj_renderer = None
        if self.sim is not None:
            try:
                self.move_joint((0.0, 0.0, 0.0, 0.0))
            except Exception:
                pass
            self.sim.close()
        self.sim = None
        self.dashboard = None
        self.move_api = None
        self.feed = None


class MG400RealClient(BaseRobotClient):
    def __init__(self, ip: str | None, robot_id: int | None, speed: int, return_home: bool):
        self.ip = ip
        self.robot_id = robot_id
        self.speed = speed
        self.return_home = return_home
        self.utils = None
        self.dashboard = None
        self.move_api = None
        self.feed = None
        self.target_ip = None

    def connect(self) -> None:
        self.utils = load_module("diagnostic_mg400_utils", MG400_UTILS)
        self.target_ip = self.utils.resolve_target_ip(ip=self.ip, robot=self.robot_id)
        self.dashboard, self.move_api, self.feed = self.utils.connect_with_diagnostics(self.target_ip)
        require_mg400_success(self.dashboard.EnableRobot(), "EnableRobot")
        time.sleep(1.5)
        self.utils.check_errors(self.dashboard)
        require_mg400_success(self.dashboard.SpeedFactor(self.speed), "SpeedFactor")
        mode = parse_mg400_mode(self.dashboard.RobotMode())
        if mode is None:
            raise ConnectionError("RobotMode response did not include a mode value.")
        if mode == 9:
            error_ids = parse_mg400_error_ids(self.dashboard.GetErrorID()) or []
            raise RuntimeError(
                f"MG400 remains in ERROR mode after startup "
                f"(error_ids={error_ids}; {self._format_error_ids(error_ids)})."
            )

    def snapshot(self) -> dict[str, Any]:
        assert self.dashboard is not None
        pose_raw = self.dashboard.GetPose()
        angle_raw = self.dashboard.GetAngle()
        mode_raw = self.dashboard.RobotMode()
        error_raw = self.dashboard.GetErrorID()
        pose = parse_mg400_pose(pose_raw)
        firmware_angles = parse_mg400_angles(angle_raw)
        return {
            "pose": pose,
            "joints": mg400_firmware_to_body(firmware_angles),
            "raw": {
                "target": "real",
                "ip": self.target_ip,
                "GetPose": str(pose_raw),
                "GetAngle": str(angle_raw),
                "RobotMode": str(mode_raw),
                "GetErrorID": str(error_raw),
                "mode": parse_mg400_mode(mode_raw),
                "error_ids": parse_mg400_error_ids(error_raw),
            },
        }

    def _check_no_error_after_sync(self) -> None:
        """Raise RuntimeError if the robot entered ERROR mode (9) after a move.

        The MG400 firmware silently rejects out-of-reach targets: MovJ/JointMovJ
        enqueues the command, Sync() returns, but the robot never moves and flips
        to mode 9. Without this check the case records 'ok' with a stale pose.
        """
        mode_raw = self.dashboard.RobotMode()
        mode = parse_mg400_mode(mode_raw)
        if mode == 9:
            error_raw = self.dashboard.GetErrorID()
            error_ids = parse_mg400_error_ids(error_raw) or []
            description = self._format_error_ids(error_ids)
            try:
                self.dashboard.ClearError()
                self.dashboard.Continue()
            except Exception:
                pass
            raise RuntimeError(
                f"Robot entered ERROR mode after move (error_ids={error_ids}; {description}). "
                "Target pose may be outside the reachable workspace or joint limits."
            )

    def _format_error_ids(self, error_ids: list[int]) -> str:
        if not error_ids:
            return "no error IDs returned"
        alarm_map = getattr(self.utils, "MG400_ALARM", {}) if self.utils is not None else {}
        parts = []
        for error_id in error_ids:
            name, fix = alarm_map.get(error_id, ("Unknown alarm", "Power-cycle the robot if it does not clear."))
            parts.append(f"{error_id}: {name}; fix: {fix}")
        return " | ".join(parts)

    def move_joint(self, q: tuple[float, float, float, float]) -> Any:
        assert self.move_api is not None
        firmware = mg400_body_to_firmware(q)
        response = require_mg400_success(self.move_api.JointMovJ(*firmware), "JointMovJ")
        sync_response = require_mg400_success(self.move_api.Sync(), "Sync")
        self._check_no_error_after_sync()
        return {"JointMovJ": response, "Sync": sync_response, "firmware_joints": firmware}

    def move_cartesian(self, pose: tuple[float, float, float, float]) -> Any:
        assert self.move_api is not None
        response = require_mg400_success(self.move_api.MovJ(*pose), "MovJ")
        sync_response = require_mg400_success(self.move_api.Sync(), "Sync")
        self._check_no_error_after_sync()
        return {"MovJ": response, "Sync": sync_response}

    def move_cartesian_linear(self, pose: tuple[float, float, float, float]) -> Any:
        assert self.move_api is not None
        response = require_mg400_success(self.move_api.MovL(*pose), "MovL")
        sync_response = require_mg400_success(self.move_api.Sync(), "Sync")
        self._check_no_error_after_sync()
        return {"MovL": response, "Sync": sync_response}

    def set_tool(self, index: int, state: bool) -> Any:
        assert self.dashboard is not None  # ToolDO is port-29999 dashboard command
        result = require_mg400_success(self.dashboard.ToolDO(index, int(bool(state))), "ToolDO")
        return {"ToolDO": result, "index": index, "state": bool(state)}

    def set_speed(self, factor: int) -> None:
        if self.dashboard is not None:
            require_mg400_success(self.dashboard.SpeedFactor(factor), "SpeedFactor")

    def start_feed(self) -> None:
        if self.feed is None or self.utils is None:
            return
        try:
            self.utils.start_feedback_thread(self.feed)
            time.sleep(0.12)  # allow first packet to arrive
        except Exception:
            pass

    def read_feed_pose(self) -> tuple[float, float, float, float] | None:
        if self.utils is None:
            return None
        try:
            cp = self.utils.current_pose
            return (float(cp["x"]), float(cp["y"]), float(cp["z"]), float(cp["r"]))
        except Exception:
            return None

    def close(self) -> None:
        if self.dashboard is None:
            return
        try:
            if self.return_home and self.utils is not None and self.move_api is not None:
                self.utils.go_home(self.move_api)
        except Exception:
            pass
        try:
            self.dashboard.DisableRobot()
        except Exception:
            pass
        try:
            if self.utils is not None:
                self.utils.close_all(self.dashboard, self.move_api, self.feed)
        finally:
            self.dashboard = None
            self.move_api = None
            self.feed = None


def build_client(args: argparse.Namespace) -> BaseRobotClient:
    if args.target == "parallel":
        raise ValueError("For parallel target use _cmd_run_parallel(); build_client() expects sim or real.")
    if args.robot == "magician" and args.target == "sim":
        return MagicianSimClient(args.backend, args.gui, args.ee_mode)
    if args.robot == "magician" and args.target == "real":
        return MagicianRealClient(args.port, not args.no_return_home)
    if args.robot == "mg400" and args.target == "sim":
        return MG400SimClient(args.backend, args.gui)
    if args.robot == "mg400" and args.target == "real":
        return MG400RealClient(args.ip, args.robot_id, args.speed, not args.no_return_home)
    raise ValueError(f"Unsupported robot/target: {args.robot}/{args.target}")


def selected_cases(args: argparse.Namespace) -> list[DiagnosticCase]:
    suites = tuple(CASES[args.robot]) if args.suite == "all" else (args.suite,)
    cases: list[DiagnosticCase] = []
    for suite in suites:
        if suite not in CASES[args.robot]:
            raise SystemExit(f"Suite {suite!r} is not available for {args.robot}.")
        cases.extend(CASES[args.robot][suite])
    if args.case:
        names = set(args.case)
        cases = [case for case in cases if case.name in names]
        missing = names - {case.name for case in cases}
        if missing:
            raise SystemExit(f"Unknown case name(s) for {args.robot}: {', '.join(sorted(missing))}")
    return cases


def validate_run_args(args: argparse.Namespace) -> None:
    if args.backend is None:
        args.backend = "mujoco"
    if args.robot == "magician" and args.backend not in MAGICIAN_BACKENDS:
        raise SystemExit(f"Magician backend must be one of: {', '.join(MAGICIAN_BACKENDS)}")
    if args.robot == "mg400" and args.backend not in MG400_BACKENDS:
        raise SystemExit(f"MG400 backend must be one of: {', '.join(MG400_BACKENDS)}")
    if args.target in ("real", "parallel") and not args.confirm_real and not args.dry_run:
        raise SystemExit("Refusing real hardware access without --confirm-real. Use --dry-run to preview commands.")
    if args.samples < 1:
        raise SystemExit("--samples must be >= 1")
    if args.speed < 1 or args.speed > 100:
        raise SystemExit("--speed must be in the range 1..100")


def run_metadata(args: argparse.Namespace, output: Path, rid: str) -> dict[str, Any]:
    return {
        "type": "run_start",
        "timestamp": utc_now(),
        "run_id": rid,
        "robot": args.robot,
        "target": args.target,
        "backend": args.backend if args.target in ("sim", "parallel") else None,
        "suite": args.suite,
        "output": str(output),
        "probe_only": bool(args.probe_only),
        "dry_run": bool(args.dry_run),
        "samples": args.samples,
        "settle_s": args.settle,
        "sample_interval_s": args.sample_interval,
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "cwd": str(Path.cwd()),
        "dobot_env": {k: v for k, v in sorted(os.environ.items()) if k.startswith("DOBOT_")},
    }


def case_preview_record(args: argparse.Namespace, case: DiagnosticCase, rid: str) -> dict[str, Any]:
    exp = expected_pose(case)
    record = {
        "type": "case_preview",
        "timestamp": utc_now(),
        "run_id": rid,
        "robot": case.robot,
        "target": args.target,
        "backend": args.backend if args.target in ("sim", "parallel") else None,
        "suite": case.suite,
        "case": case.name,
        "command": case.command,
        "values": case.values,
        "expected_pose": exp,
        "note": case.note,
    }
    if case.robot == "mg400" and case.command in ("joint", "repeat_joint", "equivalence"):
        record["firmware_joints"] = mg400_body_to_firmware(case.values)
    return record


def collect_feedback_samples(client: BaseRobotClient, args: argparse.Namespace) -> list[dict[str, Any]]:
    samples = []
    for idx in range(args.samples):
        snapshot = client.snapshot()
        snapshot["sample_index"] = idx
        snapshot["sample_timestamp"] = utc_now()
        samples.append(snapshot)
        if idx + 1 < args.samples:
            time.sleep(args.sample_interval)
    return samples


def apply_sample_fields(record: dict[str, Any], before: dict[str, Any] | None, samples: list[dict[str, Any]]) -> None:
    if not samples:
        return
    after = samples[-1]
    record["samples"] = samples
    record["after"] = after
    record["pose_after"] = after.get("pose")
    record["joints_after"] = after.get("joints")
    record["pose_error_vs_expected"] = vector_error(record.get("expected_pose"), after.get("pose"))
    if before is not None:
        record["pose_delta_from_before"] = pose_delta(before.get("pose"), after.get("pose"))
    record["sample_stats"] = sample_stats(samples)


def relative_target(before_pose: tuple[float, float, float, float], delta: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return tuple(float(a) + float(b) for a, b in zip(before_pose, delta))  # type: ignore[return-value]


def make_case_record(args: argparse.Namespace, case: DiagnosticCase, rid: str) -> dict[str, Any]:
    expected = expected_pose(case)
    record: dict[str, Any] = {
        "type": "case_result",
        "timestamp": utc_now(),
        "run_id": rid,
        "robot": case.robot,
        "target": args.target,
        "backend": args.backend if args.target in ("sim", "parallel") else None,
        "suite": case.suite,
        "case": case.name,
        "command": case.command,
        "values": case.values,
        "expected_pose": expected,
        "note": case.note,
        "status": "ok",
    }
    if case.robot == "mg400" and case.command in ("joint", "repeat_joint", "equivalence"):
        record["firmware_joints"] = mg400_body_to_firmware(case.values)
    return record


def run_case(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    rid: str,
) -> dict[str, Any]:
    started = time.monotonic()
    record = make_case_record(args, case, rid)

    try:
        before = client.snapshot()
        record["before"] = before
        if case.command == "joint":
            response = client.move_joint(case.values)
        elif case.command == "cartesian":
            response = client.move_cartesian(case.values)
        elif case.command == "relative":
            target = relative_target(before["pose"], case.values)
            record["relative_target"] = target
            record["expected_pose"] = target
            response = client.move_cartesian(target)
        elif case.command == "snapshot":
            response = {"snapshot_only": True}
        elif case.command == "io":
            index = int(case.values[0])
            state = bool(int(case.values[1]))
            record["tool_index"] = index
            record["tool_state"] = state
            response = client.set_tool(index, state)
        elif case.command == "repeat_joint":
            response = run_repeatability_sequence(client, args, case, record)
        elif case.command == "equivalence":
            response = run_equivalence_sequence(client, args, case, record)
        elif case.command == "mode_compare":
            response = run_mode_compare_sequence(client, args, case, record)
        elif case.command == "sequence":
            response = run_sequence_steps(client, args, case, record)
        elif case.command == "speed_sweep":
            response = run_speed_sweep_sequence(client, args, case, record)
        elif case.command == "feed_sample":
            response = run_feed_sample_sequence(client, args, case, record)
        else:
            raise ValueError(f"Unsupported diagnostic command: {case.command}")
        record["command_response"] = response
        if "samples" not in record:
            time.sleep(args.settle)
            apply_sample_fields(record, before, collect_feedback_samples(client, args))
    except Exception as exc:
        record["status"] = "error"
        record["error"] = f"{type(exc).__name__}: {exc}"
        record["traceback"] = traceback.format_exc()
    finally:
        record["duration_s"] = time.monotonic() - started
    return record


def run_repeatability_sequence(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    record: dict[str, Any],
) -> dict[str, Any]:
    home = (0.0, 0.0, 0.0, 0.0)
    cycle_records = []
    for cycle in range(REPEATABILITY_CYCLES):
        client.move_joint(home)
        time.sleep(args.settle)
        client.move_joint(case.values)
        time.sleep(args.settle)
        samples = collect_feedback_samples(client, args)
        after = samples[-1]
        cycle_records.append(
            {
                "cycle": cycle + 1,
                "pose": after.get("pose"),
                "joints": after.get("joints"),
                "sample_stats": sample_stats(samples),
                "pose_error_vs_expected": vector_error(record.get("expected_pose"), after.get("pose")),
            }
        )
    record["repeatability_cycles"] = REPEATABILITY_CYCLES
    record["repeatability_samples"] = cycle_records
    poses = [{"pose": item["pose"]} for item in cycle_records if item.get("pose") is not None]
    record["repeatability_stats"] = sample_stats(poses)
    apply_sample_fields(record, record.get("before"), [{"pose": cycle_records[-1]["pose"], "joints": cycle_records[-1]["joints"]}])
    return {"home": home, "target": case.values, "cycles": REPEATABILITY_CYCLES}


def run_equivalence_sequence(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    record: dict[str, Any],
) -> dict[str, Any]:
    client.move_joint(case.values)
    time.sleep(args.settle)
    joint_samples = collect_feedback_samples(client, args)
    joint_after = joint_samples[-1]
    target = record["expected_pose"]
    if case.robot == "magician" and args.target == "real":
        pose_after_joint = joint_after.get("pose")
        if pose_after_joint is not None:
            target = as_float_tuple(pose_after_joint)
            record["equivalence_cartesian_target_source"] = "joint_after_pose"
        else:
            record["equivalence_cartesian_target_source"] = "expected_pose_fallback"
    client.move_cartesian(target)
    time.sleep(args.settle)
    cart_samples = collect_feedback_samples(client, args)
    cart_after = cart_samples[-1]
    record["joint_phase"] = {"samples": joint_samples, "after": joint_after}
    record["cartesian_phase"] = {"samples": cart_samples, "after": cart_after}
    record["equivalence_error"] = vector_error(joint_after.get("pose"), cart_after.get("pose"))
    apply_sample_fields(record, record.get("before"), cart_samples)
    return {"joint_target": case.values, "cartesian_target": target}


def run_mode_compare_sequence(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    record: dict[str, Any],
) -> dict[str, Any]:
    reference = REFERENCE_POSES[case.robot]
    client.move_cartesian(reference)
    time.sleep(args.settle)
    client.move_cartesian(case.values)
    time.sleep(args.settle)
    joint_mode_samples = collect_feedback_samples(client, args)
    joint_mode_after = joint_mode_samples[-1]
    client.move_cartesian(reference)
    time.sleep(args.settle)
    linear_response = client.move_cartesian_linear(case.values)
    time.sleep(args.settle)
    linear_samples = collect_feedback_samples(client, args)
    linear_after = linear_samples[-1]
    record["joint_mode_phase"] = {"samples": joint_mode_samples, "after": joint_mode_after}
    record["linear_mode_phase"] = {"samples": linear_samples, "after": linear_after, "response": linear_response}
    record["mode_pose_difference"] = vector_error(joint_mode_after.get("pose"), linear_after.get("pose"))
    apply_sample_fields(record, record.get("before"), linear_samples)
    return {"reference_pose": reference, "target": case.values, "linear_response": linear_response}


def _dispatch_step(client: BaseRobotClient, cmd_type: str, values: Any) -> Any:
    if cmd_type == "move_joint":
        return client.move_joint(values)
    if cmd_type == "move_cartesian":
        return client.move_cartesian(values)
    if cmd_type == "move_cartesian_linear":
        return client.move_cartesian_linear(values)
    if cmd_type == "io":
        index = int(values[0])
        state = bool(int(values[1]))
        return client.set_tool(index, state)
    raise ValueError(f"Unknown step command in sequence: {cmd_type!r}")


def run_sequence_steps(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    record: dict[str, Any],
) -> dict[str, Any]:
    steps_detail = []
    last_snapshot: dict[str, Any] | None = None
    for i, (cmd_type, values) in enumerate(case.steps or ()):
        step_resp = _dispatch_step(client, cmd_type, values)
        time.sleep(args.settle)
        snap = client.snapshot()
        steps_detail.append({
            "step": i,
            "cmd_type": cmd_type,
            "values": values,
            "response": str(step_resp),
            "pose": snap.get("pose"),
        })
        last_snapshot = snap
    record["steps_detail"] = steps_detail
    if last_snapshot is not None:
        apply_sample_fields(record, record.get("before"), [last_snapshot])
    return {"steps_count": len(steps_detail)}


def run_speed_sweep_sequence(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    record: dict[str, Any],
) -> dict[str, Any]:
    reference = REFERENCE_POSES.get(case.robot, (300.0, 0.0, 50.0, 0.0))
    speed_results = []
    for speed in (case.speeds or ()):
        client.set_speed(speed)
        time.sleep(0.1)
        client.move_cartesian(reference)
        time.sleep(args.settle)
        client.move_cartesian(case.values)
        time.sleep(args.settle)
        samples = collect_feedback_samples(client, args)
        after = samples[-1]
        speed_results.append({
            "speed": speed,
            "pose_after": after.get("pose"),
            "pose_error_vs_expected": vector_error(case.values, after.get("pose")),
            "sample_stats": sample_stats(samples),
        })
    client.set_speed(20)  # reset to default
    record["speed_results"] = speed_results
    final_samples = collect_feedback_samples(client, args)
    apply_sample_fields(record, record.get("before"), final_samples)
    return {"speeds_tested": list(case.speeds or ()), "results_count": len(speed_results)}


def run_feed_sample_sequence(
    client: BaseRobotClient,
    args: argparse.Namespace,
    case: DiagnosticCase,
    record: dict[str, Any],
) -> dict[str, Any]:
    client.start_feed()
    response = client.move_cartesian(case.values)
    time.sleep(args.settle)
    feed_compare: list[dict[str, Any]] = []
    for i in range(max(case.feed_samples, 1)):
        feed_pose = client.read_feed_pose()
        snap = client.snapshot()
        dashboard_pose = snap.get("pose")
        fvd = vector_error(feed_pose, dashboard_pose) if (feed_pose and dashboard_pose) else None
        feed_compare.append({
            "i": i,
            "feed_pose": feed_pose,
            "dashboard_pose": dashboard_pose,
            "feed_vs_dashboard": fvd,
        })
        time.sleep(0.05)
    errors = [
        s["feed_vs_dashboard"]["euclidean_xyz_mm"]
        for s in feed_compare
        if s.get("feed_vs_dashboard") is not None
    ]
    max_fvd = max(errors) if errors else None
    record["feed_compare_samples"] = feed_compare
    record["max_feed_vs_dashboard_mm"] = max_fvd
    last_snap = {"pose": feed_compare[-1]["dashboard_pose"]} if feed_compare else {}
    apply_sample_fields(record, record.get("before"), [last_snap])
    return {
        "move_response": str(response),
        "feed_samples_count": len(feed_compare),
        "max_feed_vs_dashboard_mm": max_fvd,
    }


def _pose_diff(
    sim_pose: tuple[float, ...] | None,
    real_pose: tuple[float, ...] | None,
) -> dict[str, float] | None:
    if sim_pose is None or real_pose is None:
        return None
    try:
        s = tuple(float(v) for v in sim_pose)
        r = tuple(float(v) for v in real_pose)
        dx, dy, dz = s[0] - r[0], s[1] - r[1], s[2] - r[2]
        return {
            "mm": math.sqrt(dx * dx + dy * dy + dz * dz),
            "deg": abs(angle_error_deg(s[3], r[3])),
        }
    except Exception:
        return None


def _max_joint_diff(sim_joints: Any, real_joints: Any) -> float | None:
    if sim_joints is None or real_joints is None:
        return None
    try:
        sj = as_float_tuple(sim_joints)
        rj = as_float_tuple(real_joints)
        return max(abs(a - b) for a, b in zip(sj, rj))
    except Exception:
        return None


def _parallel_joint_diffs(
    case: DiagnosticCase,
    sim_result: dict[str, Any] | None,
    real_result: dict[str, Any] | None,
) -> dict[str, float | None]:
    sim_joints = sim_result.get("joints_after") if sim_result else None
    real_joints = real_result.get("joints_after") if real_result else None
    body_diff = _max_joint_diff(sim_joints, real_joints)
    firmware_diff = None
    if case.robot == "mg400" and sim_joints is not None and real_joints is not None:
        try:
            firmware_diff = _max_joint_diff(
                mg400_body_to_firmware(as_float_tuple(sim_joints)),
                mg400_body_to_firmware(as_float_tuple(real_joints)),
            )
        except Exception:
            firmware_diff = None
    return {"joint_diff_deg": body_diff, "firmware_joint_diff_deg": firmware_diff}


def _print_parallel_result(
    case: DiagnosticCase,
    sim_result: dict[str, Any] | None,
    real_result: dict[str, Any] | None,
    diff: dict[str, float] | None,
    joint_diffs: dict[str, float | None] | None = None,
) -> None:
    sim_s = (sim_result.get("status", "missing") if sim_result else "missing").upper()
    real_s = (real_result.get("status", "missing") if real_result else "missing").upper()
    diff_str = f"diff={diff['mm']:.2f}mm/{diff['deg']:.2f}deg" if diff else "diff=N/A"
    joint_str = ""
    if joint_diffs and joint_diffs.get("joint_diff_deg") is not None:
        joint_str = f" joint={joint_diffs['joint_diff_deg']:.2f}deg"
    print(f"  {case.name:<24} sim={sim_s:<5} real={real_s:<5} {diff_str}{joint_str}")


def run_parallel_cases(
    cases: list[DiagnosticCase],
    sim_client: BaseRobotClient,
    real_client: BaseRobotClient,
    sim_args: argparse.Namespace,
    real_args: argparse.Namespace,
    rid: str,
    writer: Any,
    keep_going: bool = False,
) -> int:
    """Run each case on sim and real in lockstep (two threads). Returns failure count."""
    failures = 0
    for case in cases:
        s_slot: list[Any] = [None, None]  # [result, error_str]
        r_slot: list[Any] = [None, None]

        def _run_sim(slot: list = s_slot, _case: DiagnosticCase = case) -> None:
            try:
                slot[0] = run_case(sim_client, sim_args, _case, rid)
            except Exception as exc:
                slot[1] = f"{type(exc).__name__}: {exc}"

        def _run_real(slot: list = r_slot, _case: DiagnosticCase = case) -> None:
            try:
                slot[0] = run_case(real_client, real_args, _case, rid)
            except Exception as exc:
                slot[1] = f"{type(exc).__name__}: {exc}"

        t_sim = threading.Thread(target=_run_sim, daemon=True)
        t_real = threading.Thread(target=_run_real, daemon=True)
        t_sim.start()
        t_real.start()
        t_sim.join(timeout=120)
        t_real.join(timeout=120)
        if t_sim.is_alive():
            s_slot[1] = "timeout: sim move did not complete within 120 s"
        if t_real.is_alive():
            r_slot[1] = "timeout: real move did not complete within 120 s"

        sim_result, sim_err = s_slot
        real_result, real_err = r_slot
        diff = _pose_diff(
            sim_result.get("pose_after") if sim_result else None,
            real_result.get("pose_after") if real_result else None,
        )
        joint_diffs = _parallel_joint_diffs(case, sim_result, real_result)
        ok = bool(
            sim_result and real_result
            and not sim_err and not real_err
            and sim_result.get("status") == "ok"
            and real_result.get("status") == "ok"
        )
        parallel_record: dict[str, Any] = {
            "type": "parallel_case_result",
            "timestamp": utc_now(),
            "run_id": rid,
            "case": case.name,
            "suite": case.suite,
            "robot": case.robot,
            "sim": sim_result,
            "sim_error": sim_err,
            "real": real_result,
            "real_error": real_err,
            "pose_diff_mm": diff.get("mm") if diff else None,
            "pose_diff_deg": diff.get("deg") if diff else None,
            "joint_diff_deg": joint_diffs.get("joint_diff_deg"),
            "firmware_joint_diff_deg": joint_diffs.get("firmware_joint_diff_deg"),
            "status": "ok" if ok else "error",
        }
        writer.write(parallel_record)
        _print_parallel_result(case, sim_result, real_result, diff, joint_diffs)
        if not ok:
            failures += 1
            if not keep_going:
                break
    return failures


def _cmd_run_parallel(
    args: argparse.Namespace,
    cases: list[DiagnosticCase],
    rid: str,
    output: Path,
    writer: JsonlWriter,
) -> int:
    import copy
    sim_args = copy.copy(args)
    sim_args.target = "sim"
    real_args = copy.copy(args)
    real_args.target = "real"
    sim_client = build_client(sim_args)
    real_client = build_client(real_args)
    print(f"Parallel: sim backend={args.backend}, real robot_id={args.robot_id} ip={args.ip}")
    failures = 0
    try:
        sim_client.connect()
        real_client.connect()
        failures = run_parallel_cases(
            cases, sim_client, real_client, sim_args, real_args, rid, writer,
            keep_going=args.keep_going,
        )
    finally:
        for cl in (sim_client, real_client):
            try:
                cl.close()
            except Exception:
                pass
        writer.write({
            "type": "run_end",
            "timestamp": utc_now(),
            "run_id": rid,
            "status": "failed" if failures else "ok",
            "failures": failures,
        })
    print(f"Parallel run completed with {failures} failure(s).")
    return 1 if failures else 0


def run_probe(client: BaseRobotClient, args: argparse.Namespace, rid: str) -> dict[str, Any]:
    started = time.monotonic()
    record: dict[str, Any] = {
        "type": "probe_result",
        "timestamp": utc_now(),
        "run_id": rid,
        "robot": args.robot,
        "target": args.target,
        "backend": args.backend if args.target in ("sim", "parallel") else None,
        "status": "ok",
    }
    try:
        record["snapshot"] = client.snapshot()
    except Exception as exc:
        record["status"] = "error"
        record["error"] = f"{type(exc).__name__}: {exc}"
        record["traceback"] = traceback.format_exc()
    finally:
        record["duration_s"] = time.monotonic() - started
    return record


def default_output_path(args: argparse.Namespace, rid: str) -> Path:
    return DEFAULT_RESULTS_DIR / f"{rid}.jsonl"


def print_preview(record: dict[str, Any]) -> None:
    raw_values = record.get("values")
    values = "-" if raw_values is None else ", ".join(f"{v:.1f}" for v in raw_values)
    expected_pose_value = record.get("expected_pose")
    expected = "-" if expected_pose_value is None else ", ".join(f"{v:.1f}" for v in expected_pose_value)
    print(f"  {record['case']:<18} {record['command']:<9} values=({values}) expected_pose=({expected})")


def print_case_result(record: dict[str, Any]) -> None:
    status = record.get("status", "unknown").upper()
    if status == "OK":
        err = record.get("pose_error_vs_expected") or {}
        if err:
            print(
                f"  PASS {record['case']:<18} "
                f"dxyz={err.get('max_abs_xyz_mm', float('nan')):.2f} mm "
                f"dr={err.get('rotation_deg', float('nan')):.2f} deg"
            )
        else:
            print(f"  PASS {record['case']:<18} {record.get('command', '')}")
    else:
        print(f"  FAIL {record['case']:<18} {record.get('error', 'unknown error')}")


def cmd_run(args: argparse.Namespace) -> int:
    validate_run_args(args)
    cases = selected_cases(args)
    rid = run_id(args.robot, args.target)
    output = Path(args.output).expanduser().resolve() if args.output else default_output_path(args, rid)
    print(f"Diagnostic run: robot={args.robot} target={args.target} suite={args.suite}")
    print(f"Output: {output}")

    with JsonlWriter(output) as writer:
        writer.write(run_metadata(args, output, rid))
        if args.dry_run:
            print("Dry run only. No connection and no motion.")
            for case in cases:
                record = case_preview_record(args, case, rid)
                writer.write(record)
                print_preview(record)
            writer.write({"type": "run_end", "timestamp": utc_now(), "run_id": rid, "status": "dry_run"})
            return 0

        if args.target == "parallel":
            return _cmd_run_parallel(args, cases, rid, output, writer)

        client = build_client(args)
        failures = 0
        try:
            client.connect()
            if args.probe_only:
                record = run_probe(client, args, rid)
                writer.write(record)
                print(f"Probe status: {record['status']}")
                if record["status"] != "ok":
                    failures += 1
            else:
                for case in cases:
                    record = run_case(client, args, case, rid)
                    writer.write(record)
                    print_case_result(record)
                    if record.get("status") != "ok":
                        failures += 1
                        if not args.keep_going:
                            break
        finally:
            close_error = None
            try:
                client.close()
            except Exception as exc:
                close_error = f"{type(exc).__name__}: {exc}"
                failures += 1
            writer.write(
                {
                    "type": "run_end",
                    "timestamp": utc_now(),
                    "run_id": rid,
                    "status": "failed" if failures else "ok",
                    "failures": failures,
                    "close_error": close_error,
                }
            )

    print(f"Completed with {failures} failure(s).")
    return 1 if failures else 0


def iter_jsonl(path: Path) -> list[dict[str, Any]]:
    records = []
    with path.open("r", encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{lineno}: invalid JSON: {exc}") from exc
    return records


def case_key(record: dict[str, Any]) -> tuple[str, str, str]:
    return str(record.get("robot")), str(record.get("suite")), str(record.get("case"))


def successful_case_records(records: list[dict[str, Any]]) -> dict[tuple[str, str, str], dict[str, Any]]:
    result = {}
    for record in records:
        if record.get("type") == "case_result" and record.get("status") == "ok":
            result[case_key(record)] = record
    return result


def compare_pose_records(sim_record: dict[str, Any], real_record: dict[str, Any]) -> dict[str, Any]:
    sim_pose = as_float_tuple(sim_record.get("pose_after"))
    real_pose = as_float_tuple(real_record.get("pose_after"))
    sim_joints = sim_record.get("joints_after")
    real_joints = real_record.get("joints_after")
    pos_err = max_position_error(sim_pose, real_pose)
    rot_err = angle_error_deg(sim_pose[3], real_pose[3])
    joint_err = None
    if sim_joints is not None and real_joints is not None:
        sj = as_float_tuple(sim_joints)
        rj = as_float_tuple(real_joints)
        joint_err = max(abs(a - b) for a, b in zip(sj, rj))
    return {
        "sim_pose": sim_pose,
        "real_pose": real_pose,
        "max_position_error_mm": pos_err,
        "rotation_error_deg": rot_err,
        "max_joint_error_deg": joint_err,
    }


def _xyz_delta(sim_pose: tuple[float, ...], real_pose: tuple[float, ...]) -> tuple[float, float, float]:
    return (
        float(sim_pose[0] - real_pose[0]),
        float(sim_pose[1] - real_pose[1]),
        float(sim_pose[2] - real_pose[2]),
    )


def _resolve_magician_anchor_key(
    common: list[tuple[str, str, str]],
    preferred_case: str,
) -> tuple[str, str, str] | None:
    magician_keys = [key for key in common if key[0] == "magician"]
    if not magician_keys:
        return None
    for key in magician_keys:
        if key[2] == preferred_case:
            return key
    for key in magician_keys:
        if key[1] == "joint":
            return key
    return magician_keys[0]


def _magician_hybrid_metrics(
    diff: dict[str, Any],
    baseline_xyz: tuple[float, float, float] | None,
) -> dict[str, Any]:
    sim_pose = diff["sim_pose"]
    real_pose = diff["real_pose"]
    dx, dy, dz = _xyz_delta(sim_pose, real_pose)
    bx, by, bz = baseline_xyz if baseline_xyz is not None else (0.0, 0.0, 0.0)
    ndx = dx - bx
    ndy = dy - by
    ndz = dz - bz
    return {
        "raw_dx_mm": dx,
        "raw_dy_mm": dy,
        "raw_dz_mm": dz,
        "norm_dx_mm": ndx,
        "norm_dy_mm": ndy,
        "norm_dz_mm": ndz,
        "norm_max_position_error_mm": max(abs(ndx), abs(ndy), abs(ndz)),
    }


def cmd_compare(args: argparse.Namespace) -> int:
    left_path = Path(args.sim_jsonl).expanduser().resolve()
    right_path = Path(args.real_jsonl).expanduser().resolve()
    left = successful_case_records(iter_jsonl(left_path))
    right = successful_case_records(iter_jsonl(right_path))
    common = sorted(set(left) & set(right))
    if not common:
        print("No matching successful case_result records found.")
        return 1

    failures = 0
    max_pos = 0.0
    max_rot = 0.0
    max_norm_pos = 0.0
    print(f"Comparing {len(common)} matched case(s)")
    print(f"Tolerances: {args.pos_tol:.1f} mm, {args.rot_tol:.1f} deg")
    use_magician_hybrid = bool(args.magician_hybrid)
    baseline_xyz: tuple[float, float, float] | None = None
    anchor_key: tuple[str, str, str] | None = None
    if use_magician_hybrid:
        anchor_key = _resolve_magician_anchor_key(common, args.anchor_case)
        if anchor_key is None:
            print("Hybrid mode requested but no matched Magician records were found; using default compare scoring.")
            use_magician_hybrid = False
        else:
            anchor_diff = compare_pose_records(left[anchor_key], right[anchor_key])
            baseline_xyz = _xyz_delta(anchor_diff["sim_pose"], anchor_diff["real_pose"])
            print(
                "Magician hybrid mode: baseline XYZ offset "
                f"({baseline_xyz[0]:.2f}, {baseline_xyz[1]:.2f}, {baseline_xyz[2]:.2f}) mm "
                f"from case {anchor_key[1]}/{anchor_key[2]}."
            )
            print(
                f"Hybrid tolerances: normalized XYZ <= {args.pos_tol:.1f} mm, "
                f"joint <= {args.joint_tol:.1f} deg (R ignored)."
            )
    for key in common:
        diff = compare_pose_records(left[key], right[key])
        pos = diff["max_position_error_mm"]
        rot = diff["rotation_error_deg"]
        max_pos = max(max_pos, pos)
        max_rot = max(max_rot, rot)
        robot, suite, case = key
        hybrid_metrics = None
        if use_magician_hybrid and robot == "magician":
            hybrid_metrics = _magician_hybrid_metrics(diff, baseline_xyz)
            npos = hybrid_metrics["norm_max_position_error_mm"]
            max_norm_pos = max(max_norm_pos, npos)
            joint_err = diff["max_joint_error_deg"]
            ok = npos <= args.pos_tol and joint_err is not None and joint_err <= args.joint_tol
        else:
            ok = pos <= args.pos_tol and rot <= args.rot_tol
        failures += int(not ok)
        tag = "PASS" if ok else "FAIL"
        joint_part = ""
        if diff["max_joint_error_deg"] is not None:
            joint_part = f" joint={diff['max_joint_error_deg']:.2f} deg"
        if hybrid_metrics is not None:
            print(
                f"  {tag} {robot:<8} {suite:<9} {case:<18} "
                f"dxyz={pos:.2f} mm dr={rot:.2f} deg "
                f"norm_xyz={hybrid_metrics['norm_max_position_error_mm']:.2f} mm{joint_part}"
            )
        else:
            print(f"  {tag} {robot:<8} {suite:<9} {case:<18} dxyz={pos:.2f} mm dr={rot:.2f} deg{joint_part}")

    missing_left = sorted(set(right) - set(left))
    missing_right = sorted(set(left) - set(right))
    if missing_left:
        print(f"Cases only in real file or failed in sim file: {len(missing_left)}")
    if missing_right:
        print(f"Cases only in sim file or failed in real file: {len(missing_right)}")
    if use_magician_hybrid:
        print(f"Max normalized XYZ difference (Magician hybrid): {max_norm_pos:.2f} mm")
    print(f"Max difference: {max_pos:.2f} mm, {max_rot:.2f} deg")
    return 1 if failures else 0


def cmd_list(args: argparse.Namespace) -> int:
    robots = ("magician", "mg400") if args.robot == "both" else (args.robot,)
    records = []
    for robot in robots:
        suites = tuple(CASES[robot]) if args.suite == "all" else (args.suite,)
        for suite in suites:
            if suite not in CASES[robot]:
                continue
            for case in CASES[robot][suite]:
                record = {
                    "robot": robot,
                    "suite": suite,
                    "case": case.name,
                    "command": case.command,
                    "values": case.values,
                    "expected_pose": expected_pose(case),
                    "note": case.note,
                }
                if robot == "mg400" and case.command in ("joint", "repeat_joint", "equivalence"):
                    record["firmware_joints"] = mg400_body_to_firmware(case.values)
                records.append(record)
    if args.json:
        print(json.dumps(records, indent=2, sort_keys=True))
        return 0
    for record in records:
        expected_pose_value = record.get("expected_pose")
        expected = "-" if expected_pose_value is None else ", ".join(f"{v:.1f}" for v in expected_pose_value)
        raw_values = record.get("values")
        values = "-" if raw_values is None else ", ".join(f"{v:.1f}" for v in raw_values)
        print(f"{record['robot']:<8} {record['suite']:<12} {record['case']:<24} {record['command']:<12} values=({values}) expected=({expected})")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run safe real-vs-simulation diagnostic command sets and compare outputs."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="Run diagnostics against simulation or real hardware")
    run.add_argument("--robot", required=True, choices=("magician", "mg400"), help="Robot family to test")
    run.add_argument("--target", required=True, choices=("sim", "real", "parallel"),
                     help="Run against simulation, real hardware, or both in lockstep (parallel)")
    run.add_argument("--suite", default="joint", choices=SUITE_CHOICES, help="Command suite to run")
    run.add_argument("--case", action="append", help="Run one named case. Repeat for multiple cases")
    run.add_argument("--backend", help="Simulation backend. Default: mujoco")
    run.add_argument(
        "--ee-mode",
        default=os.environ.get("DOBOT_EE", "none"),
        choices=("none", "motor", "suction"),
        help="Magician simulation EE mode (none|motor|suction). Default: DOBOT_EE or none.",
    )
    run.add_argument("--gui", action="store_true", help="Open simulator GUI when target=sim")
    run.add_argument("--confirm-real", action="store_true", help="Required for target=real unless --dry-run is used")
    run.add_argument("--dry-run", action="store_true", help="Write preview records without connecting or moving")
    run.add_argument("--probe-only", action="store_true", help="Connect and record one feedback snapshot without motion")
    run.add_argument("--output", help="JSONL output path. Defaults to simulation/diagnostics/results/<run_id>.jsonl")
    run.add_argument("--samples", type=int, default=3, help="Feedback samples after each command")
    run.add_argument("--settle", type=float, default=0.2, help="Seconds to wait after each command before sampling")
    run.add_argument("--sample-interval", type=float, default=0.05, help="Seconds between feedback samples")
    run.add_argument("--keep-going", action="store_true", help="Continue after a case error")
    run.add_argument("--no-return-home", action="store_true", help="Do not send the default return-home command on close")
    run.add_argument("--port", help="Magician serial port override for target=real")
    run.add_argument("--ip", help="MG400 IP override for target=real")
    run.add_argument("--robot-id", type=int, choices=(1, 2, 3, 4), help="MG400 robot number for target=real")
    run.add_argument("--speed", type=int, default=20, help="MG400 real hardware SpeedFactor percentage")
    run.set_defaults(func=cmd_run)

    compare = sub.add_parser("compare", help="Compare simulation JSONL against real JSONL")
    compare.add_argument("sim_jsonl", help="Simulation run JSONL")
    compare.add_argument("real_jsonl", help="Real robot run JSONL")
    compare.add_argument("--pos-tol", type=float, default=10.0, help="Maximum allowed XYZ difference in mm")
    compare.add_argument("--rot-tol", type=float, default=5.0, help="Maximum allowed R difference in degrees")
    compare.add_argument(
        "--magician-hybrid",
        action="store_true",
        help="Magician-only compare policy: baseline-offset XYZ, ignore R, joint-first pass criteria.",
    )
    compare.add_argument(
        "--anchor-case",
        default="joint_zero",
        help="Case name used as Magician baseline offset anchor when --magician-hybrid is enabled.",
    )
    compare.add_argument(
        "--joint-tol",
        type=float,
        default=5.0,
        help="Maximum allowed joint error in degrees for --magician-hybrid pass/fail scoring.",
    )
    compare.set_defaults(func=cmd_compare)

    list_cmd = sub.add_parser("list", help="List built-in diagnostic cases")
    list_cmd.add_argument("--robot", default="both", choices=("both", "magician", "mg400"), help="Robot family")
    list_cmd.add_argument("--suite", default="all", choices=SUITE_CHOICES, help="Suite to list")
    list_cmd.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    list_cmd.set_defaults(func=cmd_list)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
