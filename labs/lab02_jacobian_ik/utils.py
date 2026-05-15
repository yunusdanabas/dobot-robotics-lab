"""
Robot helpers for ME403 Lab 02 (Jacobian / IK).

Self-contained lab folder: simulation on by default; no legacy shared student helpers.

Use setup(), move_joints(robot, q), get_pose(robot), get_joints(robot),
move_and_get_feedback(robot, q), teardown(robot). moveMagician and moveMG400
alias move_joints.

Magician: setup() goes to joint home (0,0,0,0). MG400: setup() and teardown()
use the Cartesian ready pose in ROBOT_MODELS.

Env: DOBOT_ROBOT_TYPE, DOBOT_SIMULATION, DOBOT_SIM_BACKEND, or edit ROBOT_TYPE below.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import re
import struct
import sys
import time
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# Student configuration
# ---------------------------------------------------------------------------

ROBOT_TYPE = os.environ.get("DOBOT_ROBOT_TYPE", "magician").lower()  # "magician" or "mg400"
USE_SIMULATION = os.environ.get("DOBOT_SIMULATION", "1").lower() not in {"0", "false", "no", "off"}
SIM_BACKEND = os.environ.get("DOBOT_SIM_BACKEND", "mujoco").lower()  # "mujoco" or "pybullet"

MG400_IP = os.environ.get("DOBOT_MG400_IP", os.environ.get("MG400_IP", ""))
MG400_ROBOT = int(os.environ.get("DOBOT_MG400_ROBOT", "1"))


# ---------------------------------------------------------------------------
# Robot constants
# ---------------------------------------------------------------------------

ROBOT_IPS = {
    1: "192.168.2.7",
    2: "192.168.2.10",
    3: "192.168.2.9",
    4: "192.168.2.6",
}

ROBOT_MODELS = {
    "magician": {
        "name": "Dobot Magician",
        "L1": 135.0,
        "L2": 147.0,
        "Z_base": 103.0,
        "ready_pose": (200.0, 0.0, 100.0, 0.0),
        "joint_bounds_fw": {
            "j1": (-90.0, 90.0),
            "j2": (0.0, 85.0),
            "j3": (-10.0, 85.0),
            "j4": (-90.0, 90.0),
        },
        "safe_bounds": {
            "x": (120.0, 315.0),
            "y": (-158.0, 158.0),
            "z": (5.0, 155.0),
            "r": (-90.0, 90.0),
        },
    },
    "mg400": {
        "name": "DOBOT MG400",
        "L1": 175.0,
        "L2": 175.0,
        "Z_base": 116.0,
        "ready_pose": (300.0, 0.0, 50.0, 0.0),
        "joint_bounds_fw": {
            "j1": (-160.0, 160.0),
            "j2": (-25.0, 85.0),
            "j3": (-25.0, 105.0),
            "j4": (-180.0, 180.0),
        },
        "safe_bounds": {
            "x": (60.0, 400.0),
            "y": (-220.0, 220.0),
            "z": (5.0, 140.0),
            "r": (-170.0, 170.0),
        },
    },
}

L1 = L2 = Z_base = 0.0
READY_POSE: tuple[float, float, float, float]
JOINT_BOUNDS_FW: dict[str, tuple[float, float]]
SAFE_BOUNDS: dict[str, tuple[float, float]]

# MG400 pose model aligned with real/sim GetPose (same as simulation/runtime/
# kinematics.py fk_mg400_api). Use MG400_API_* in FK/Jacobian vs get_pose.
MG400_API_L1 = 174.5
MG400_API_L2 = 175.5
MG400_API_C_R = 108.0
MG400_API_C_Z = -52.0

# ---------------------------------------------------------------------------
# Magician tool / end-effector offset API
# ---------------------------------------------------------------------------

# Magician TCP presets (mm). MG400 path uses bare TCP; this block is Magician-only.

MAGICIAN_TOOL_OFFSETS: dict[str, tuple[float, float, float]] = {
    "none":    (0.0,  0.0, 0.0),   # bare flange  -> home TCP (147, 0, 135) mm
    "motor":   (60.0, 0.0, 0.0),   # motor flange -> home TCP (207, 0, 135) mm
    "suction": (60.0, 0.0, -70.0),  # suction cup  -> home TCP (207, 0, 65) mm (physical cup tip)
}


def set_tool_offset(bot, ox: float, oy: float, oz: float) -> None:
    """Write TCP offset (mm) to Magician firmware (protocol 60).

    Matches sim DOBOT_EE to real TCP so FK/Jacobians use one frame. Magician only.
    """
    from pydobotplus.dobotplus import Message
    msg = Message()
    msg.id = 60
    msg.ctrl = 0x03
    msg.params = bytearray(
        struct.pack('f', ox) + struct.pack('f', oy) + struct.pack('f', oz)
    )
    bot._send_command(msg)


def get_tool_offset(bot) -> tuple[float, float, float]:
    """Read current TCP offset in mm from Magician firmware."""
    from pydobotplus.dobotplus import Message
    msg = Message()
    msg.id = 60
    msg.ctrl = 0x00
    resp = bot._send_command(msg)
    ox = struct.unpack_from('f', resp.params, 0)[0]
    oy = struct.unpack_from('f', resp.params, 4)[0]
    oz = struct.unpack_from('f', resp.params, 8)[0]
    return float(ox), float(oy), float(oz)


@dataclass
class RobotSession:
    """One connected robot or sim instance plus handles."""
    type: str
    simulation: bool
    backend: str | None = None
    handles: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.handles is None:
            self.handles = {}


_HERE = Path(__file__).resolve().parent


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


# Sim-only joint interpolation timing (matches sim_dobot / sim_mg400).
_SIM_INTERPOLATE_STEPS = 30
_SIM_INTERPOLATE_PAUSE_S = 0.02


def _magician_max_joint_err_deg(bot, fw_target: tuple[float, float, float, float]) -> float:
    *_, j1, j2, j3, j4 = _unpack_pose(bot.get_pose())
    return max(abs(a - b) for a, b in zip((j1, j2, j3, j4), fw_target))


def _body_max_joint_err_deg(robot: RobotSession, q_target: list[float]) -> float:
    cur = get_joints(robot)
    return max(abs(c - float(t)) for c, t in zip(cur, q_target))


_JOINT_SKIP_TOL_DEG = 0.12


def _normalise_robot_type(robot_type: str) -> str:
    value = robot_type.strip().lower()
    if value not in ROBOT_MODELS:
        raise ValueError(f"Unknown ROBOT_TYPE={robot_type!r}. Expected 'magician' or 'mg400'.")
    return value


def _sync_robot_constants() -> None:
    global L1, L2, Z_base, READY_POSE, JOINT_BOUNDS_FW, SAFE_BOUNDS
    model = ROBOT_MODELS[_normalise_robot_type(ROBOT_TYPE)]
    L1 = model["L1"]
    L2 = model["L2"]
    Z_base = model["Z_base"]
    READY_POSE = model["ready_pose"]
    JOINT_BOUNDS_FW = model["joint_bounds_fw"]
    SAFE_BOUNDS = model["safe_bounds"]


def configure(
    *,
    robot_type: str | None = None,
    use_simulation: bool | None = None,
    sim_backend: str | None = None,
    mg400_ip: str | None = None,
    mg400_robot: int | None = None,
) -> None:
    """Read DOBOT_* env and optional kwargs into module globals; call before setup()."""
    global ROBOT_TYPE, USE_SIMULATION, SIM_BACKEND, MG400_IP, MG400_ROBOT

    ROBOT_TYPE = _normalise_robot_type(os.environ.get("DOBOT_ROBOT_TYPE", robot_type or ROBOT_TYPE))
    USE_SIMULATION = _env_bool("DOBOT_SIMULATION", USE_SIMULATION if use_simulation is None else use_simulation)
    SIM_BACKEND = os.environ.get("DOBOT_SIM_BACKEND", sim_backend or SIM_BACKEND).strip().lower()
    MG400_IP = os.environ.get("DOBOT_MG400_IP", os.environ.get("MG400_IP", mg400_ip or MG400_IP))
    MG400_ROBOT = int(os.environ.get("DOBOT_MG400_ROBOT", str(mg400_robot or MG400_ROBOT)))
    _sync_robot_constants()


_sync_robot_constants()


# ---------------------------------------------------------------------------
# Path and parsing helpers
# ---------------------------------------------------------------------------

def _ensure_simulation_path() -> Path:
    candidates = []
    env_runtime = os.environ.get("DOBOT_SIM_RUNTIME")
    if env_runtime:
        candidates.append(Path(env_runtime).expanduser())
    candidates.append(_HERE / "simulation" / "runtime")
    candidates.extend(parent / "simulation" / "runtime" for parent in _HERE.parents)
    for candidate in candidates:
        if (candidate / "sim_dobot.py").exists() and (candidate / "sim_mg400.py").exists():
            if str(candidate) not in sys.path:
                sys.path.insert(0, str(candidate))
            return candidate
    raise ImportError(
        "Full simulation runtime not found. From ME403_LabFiles/, run:\n"
        "  python scripts/bootstrap.py --simulation\n"
        "Or set DOBOT_SIM_RUNTIME=/path/to/simulation/runtime."
    )


def _find_dobot_api() -> Path:
    candidates: list[Path] = []
    for base in [_HERE, *_HERE.parents]:
        candidates.append(base / "dobot_api.py")
        candidates.append(base / "vendor" / "TCP-IP-4Axis-Python" / "dobot_api.py")
    for candidate in candidates:
        if candidate.exists():
            return candidate.parent
    raise ImportError(
        "dobot_api.py not found. For real MG400 use, clone the SDK into this lab folder:\n"
        "  git clone https://github.com/Dobot-Arm/TCP-IP-4Axis-Python.git "
        "vendor/TCP-IP-4Axis-Python\n"
        "Simulation mode does not require the SDK."
    )


def _parse_number_tuple(response: str, expected: int, label: str) -> tuple[float, ...]:
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", str(response))
    values = [float(n) for n in nums]
    if len(values) >= expected + 1:
        return tuple(values[1:1 + expected])
    if len(values) >= expected:
        return tuple(values[:expected])
    raise ValueError(f"Cannot parse {label} from: {response!r}")


def _unpack_pose(pose) -> tuple[float, float, float, float, float, float, float, float]:
    if hasattr(pose, "position") and hasattr(pose, "joints"):
        return (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z), float(pose.position.r),
            float(pose.joints.j1), float(pose.joints.j2),
            float(pose.joints.j3), float(pose.joints.j4),
        )
    if isinstance(pose, (tuple, list)) and len(pose) == 8:
        return tuple(float(v) for v in pose)
    if isinstance(pose, (tuple, list)) and len(pose) == 4:
        x, y, z, r = (float(v) for v in pose)
        return x, y, z, r, 0.0, 0.0, 0.0, 0.0
    raise ValueError(f"Unsupported pose format: {type(pose)!r}")


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _clamp_tuple(values: tuple[float, float, float, float], bounds: dict[str, tuple[float, float]]):
    keys = ("j1", "j2", "j3", "j4")
    return tuple(_clamp(value, *bounds[key]) for value, key in zip(values, keys))


# ---------------------------------------------------------------------------
# Joint conversion
# ---------------------------------------------------------------------------

def body_to_firmware_angles(q, robot_type: str | None = None) -> tuple[float, float, float, float]:
    """Map body-frame q1..q4 to firmware joint angles."""
    if len(q) != 4:
        raise ValueError(f"q must have 4 elements, got {len(q)}")
    rt = _normalise_robot_type(robot_type or ROBOT_TYPE)
    q1, q2, q3, q4 = (float(v) for v in q)
    if rt == "magician":
        return q1, q2, q3, q4
    return q1, q2, q2 + q3, q4


def firmware_to_body_angles(joints, robot_type: str | None = None) -> tuple[float, float, float, float]:
    """Map firmware angles to body-frame q1..q4."""
    rt = _normalise_robot_type(robot_type or ROBOT_TYPE)
    j1, j2, j3, j4 = (float(v) for v in joints)
    if rt == "magician":
        return j1, j2, j3, j4
    return j1, j2, j3 - j2, j4


# ---------------------------------------------------------------------------
# Magician adapter
# ---------------------------------------------------------------------------

class _FallbackMOVJAngleMode:
    name = "MOVJ_ANGLE"

    def __str__(self) -> str:
        return self.name


def _movj_angle_mode():
    try:
        from pydobotplus.dobotplus import MODE_PTP
        return MODE_PTP.MOVJ_ANGLE
    except Exception:
        return _FallbackMOVJAngleMode()


def _patch_pydobotplus() -> None:
    try:
        import pydobotplus.dobotplus as _dp
        from pydobotplus.dobotplus import MODE_PTP

        original = _dp.Dobot.move_to

        def fast_move_to(self, x=None, y=None, z=None, r=0, wait=True, mode=None, position=None):
            if position is not None:
                x, y, z, r = position.x, position.y, position.z, position.r
            if x is None or y is None or z is None:
                return original(self, x=x, y=y, z=z, r=r, wait=wait, mode=mode)
            if mode is None:
                mode = MODE_PTP.MOVJ_XYZ
            return self._extract_cmd_index(self._set_ptp_cmd(x, y, z, r, mode, wait=wait))

        _dp.Dobot.move_to = fast_move_to
    except Exception:
        pass


def find_port() -> str | None:
    preferred = os.environ.get("DOBOT_PORT")
    if preferred:
        return preferred
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    for port in ports:
        text = f"{port.description or ''} {port.hwid}".lower()
        if any(token in text for token in ("silicon labs", "1a86", "usb2.0-serial", "cp210", "ch340")):
            return port.device
    return ports[0].device if ports else None


def _setup_magician() -> RobotSession:
    if USE_SIMULATION:
        _ensure_simulation_path()
        from sim_dobot import SimDobot

        print(f"[setup] Dobot Magician simulation, backend={SIM_BACKEND}")
        bot = SimDobot(backend=SIM_BACKEND, tool=os.environ.get("DOBOT_EE", "none"))
        if hasattr(bot, "interpolate_joints_to"):
            bot.interpolate_joints_to(
                [0.0, 0.0, 0.0, 0.0], steps=_SIM_INTERPOLATE_STEPS, pause=_SIM_INTERPOLATE_PAUSE_S
            )
        else:
            bot.move_to(0, 0, 0, 0, wait=True, mode=_movj_angle_mode())
        return RobotSession("magician", simulation=True, backend=SIM_BACKEND, handles={"bot": bot})

    _patch_pydobotplus()
    from pydobotplus import Dobot

    port = find_port()
    if port is None:
        raise OSError("No Dobot Magician serial port found. Set DOBOT_PORT to override auto-detect.")
    print(f"[setup] Connecting to Dobot Magician on {port}")
    bot = Dobot(port=port)
    if bot.get_alarms():
        bot.clear_alarms()
    bot.move_to(0, 0, 0, 0, wait=True, mode=_movj_angle_mode())
    return RobotSession("magician", simulation=False, handles={"bot": bot})


# ---------------------------------------------------------------------------
# MG400 adapter
# ---------------------------------------------------------------------------

def _resolve_mg400_ip() -> str:
    env_ip = os.environ.get("DOBOT_MG400_IP") or os.environ.get("MG400_IP")
    if env_ip:
        return env_ip
    if MG400_IP:
        return MG400_IP
    return ROBOT_IPS.get(MG400_ROBOT, ROBOT_IPS[1])


def _setup_mg400() -> RobotSession:
    if USE_SIMULATION:
        _ensure_simulation_path()
        from sim_mg400 import SimMG400

        print(f"[setup] MG400 simulation, backend={SIM_BACKEND}")
        sim = SimMG400(backend=SIM_BACKEND)
        dashboard, move_api, feed = sim.connect()
        dashboard.EnableRobot()
        move_api.MovJ(*ROBOT_MODELS["mg400"]["ready_pose"])
        move_api.Sync()
        return RobotSession(
            "mg400",
            simulation=True,
            backend=SIM_BACKEND,
            handles={"dashboard": dashboard, "move_api": move_api, "feed": feed, "sim": sim},
        )

    sdk_dir = _find_dobot_api()
    if str(sdk_dir) not in sys.path:
        sys.path.insert(0, str(sdk_dir))
    from dobot_api import DobotApiDashboard, DobotApiMove

    ip = _resolve_mg400_ip()
    print(f"[setup] Connecting to MG400 at {ip}")
    dashboard = DobotApiDashboard(ip, 29999)
    move_api = DobotApiMove(ip, 30003)
    dashboard.EnableRobot()
    time.sleep(1.0)
    try:
        err = dashboard.GetErrorID()
        if "{}" not in str(err) and "0,{}" not in str(err):
            dashboard.ClearError()
            dashboard.Continue()
    except Exception:
        pass
    try:
        dashboard.SpeedFactor(30)
    except Exception:
        pass
    move_api.MovJ(*ROBOT_MODELS["mg400"]["ready_pose"])
    move_api.Sync()
    return RobotSession("mg400", simulation=False, handles={"dashboard": dashboard, "move_api": move_api})


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def setup() -> RobotSession:
    """Connect or start sim: Magician to joint home, MG400 to ready pose."""
    configure()
    print(
        f"[setup] robot={ROBOT_TYPE} simulation={USE_SIMULATION} "
        f"backend={SIM_BACKEND if USE_SIMULATION else 'real'}"
    )
    if ROBOT_TYPE == "magician":
        return _setup_magician()
    return _setup_mg400()


def get_pose(robot: RobotSession) -> tuple[float, float, float, float]:
    """Current TCP pose x, y, z (mm) and r (deg)."""
    if robot.type == "magician":
        x, y, z, r, *_ = _unpack_pose(robot.handles["bot"].get_pose())
        return x, y, z, r
    return _parse_number_tuple(robot.handles["dashboard"].GetPose(), 4, "MG400 pose")


def get_joints(robot: RobotSession) -> tuple[float, float, float, float]:
    """Body-frame q1..q4 in degrees."""
    if robot.type == "magician":
        *_, j1, j2, j3, j4 = _unpack_pose(robot.handles["bot"].get_pose())
        return firmware_to_body_angles((j1, j2, j3, j4), "magician")
    joints_fw = _parse_number_tuple(robot.handles["dashboard"].GetAngle(), 4, "MG400 angles")
    return firmware_to_body_angles(joints_fw, "mg400")


def move_joints(robot: RobotSession, q) -> None:
    """Move to body-frame q = [q1,q2,q3,q4] in degrees."""
    fw = body_to_firmware_angles(q, robot.type)
    bounds = ROBOT_MODELS[robot.type]["joint_bounds_fw"]
    clamped = _clamp_tuple(fw, bounds)
    if clamped != fw:
        print(
            "[move_joints] Motion denied: firmware angles exceed safe limits. "
            f"requested={tuple(round(v, 2) for v in fw)} "
            f"limits={bounds}"
        )
        return

    print(f"[move_joints] body={tuple(round(float(v), 2) for v in q)} fw={tuple(round(v, 2) for v in fw)}")
    q_body = [float(v) for v in q]
    sim_interp = robot.simulation

    if robot.type == "magician":
        bot = robot.handles["bot"]
        if sim_interp and hasattr(bot, "interpolate_joints_to") and _magician_max_joint_err_deg(bot, fw) > _JOINT_SKIP_TOL_DEG:
            bot.interpolate_joints_to(list(fw), steps=_SIM_INTERPOLATE_STEPS, pause=_SIM_INTERPOLATE_PAUSE_S)
        else:
            bot.move_to(*fw, wait=True, mode=_movj_angle_mode())
        return

    move_api = robot.handles["move_api"]
    if sim_interp and hasattr(move_api, "interpolate_joints_to") and _body_max_joint_err_deg(robot, q_body) > _JOINT_SKIP_TOL_DEG:
        move_api.interpolate_joints_to(q_body, steps=_SIM_INTERPOLATE_STEPS, pause=_SIM_INTERPOLATE_PAUSE_S)
        if hasattr(move_api, "Sync"):
            move_api.Sync()
    else:
        move_api.JointMovJ(*fw)
        move_api.Sync()


def move_and_get_feedback(robot: RobotSession, q) -> tuple[float, float, float, float]:
    """move_joints then return get_pose."""
    move_joints(robot, q)
    return get_pose(robot)


def safe_move(robot: RobotSession, x: float, y: float, z: float, r: float, mode: str = "J") -> None:
    """Cartesian move to x,y,z,r after clamping to safe_bounds."""
    bounds = ROBOT_MODELS[robot.type]["safe_bounds"]
    cx = _clamp(float(x), *bounds["x"])
    cy = _clamp(float(y), *bounds["y"])
    cz = _clamp(float(z), *bounds["z"])
    cr = _clamp(float(r), *bounds["r"])
    if (cx, cy, cz, cr) != (x, y, z, r):
        print(f"[safe_move] Clamped ({x:.1f},{y:.1f},{z:.1f},{r:.1f}) -> ({cx:.1f},{cy:.1f},{cz:.1f},{cr:.1f})")
    if robot.type == "magician":
        robot.handles["bot"].move_to(cx, cy, cz, cr, wait=True)
        return
    move_api = robot.handles["move_api"]
    if mode.upper() == "L":
        move_api.MovL(cx, cy, cz, cr)
    else:
        move_api.MovJ(cx, cy, cz, cr)
    move_api.Sync()


def teardown(robot: RobotSession) -> None:
    """Magician: joint home. MG400: interpolate in sim then MovJ ready pose. Then disconnect."""
    sim_interp = getattr(robot, "simulation", False)
    try:
        if robot.type == "magician":
            bot = robot.handles["bot"]
            home_fw = (0.0, 0.0, 0.0, 0.0)
            if sim_interp and hasattr(bot, "interpolate_joints_to") and _magician_max_joint_err_deg(bot, home_fw) > _JOINT_SKIP_TOL_DEG:
                bot.interpolate_joints_to(
                    [0.0, 0.0, 0.0, 0.0], steps=_SIM_INTERPOLATE_STEPS, pause=_SIM_INTERPOLATE_PAUSE_S
                )
            else:
                bot.move_to(0, 0, 0, 0, wait=True, mode=_movj_angle_mode())
        else:
            mv = robot.handles["move_api"]
            if sim_interp and hasattr(mv, "interpolate_joints_to"):
                # Near-neutral joints in sim, then Cartesian ready pose
                mv.interpolate_joints_to(
                    [0.0, 0.0, 0.0, 0.0], steps=_SIM_INTERPOLATE_STEPS, pause=_SIM_INTERPOLATE_PAUSE_S
                )
                if hasattr(mv, "Sync"):
                    mv.Sync()
            mv.MovJ(*ROBOT_MODELS["mg400"]["ready_pose"])
            mv.Sync()
    except Exception:
        pass

    if robot.type == "magician":
        try:
            robot.handles["bot"].close()
        except Exception:
            pass
    else:
        if robot.simulation:
            try:
                robot.handles["sim"].close()
            except Exception:
                pass
        else:
            try:
                robot.handles["dashboard"].DisableRobot()
            except Exception:
                pass
            for key in ("dashboard", "move_api"):
                try:
                    robot.handles[key].close()
                except Exception:
                    pass
    print("[teardown] Done.")


# Aliases for older handouts.
moveMagician = move_joints
moveMG400 = move_joints
move_cartesian = safe_move
