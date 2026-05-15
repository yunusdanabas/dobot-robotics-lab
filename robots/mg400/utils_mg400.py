"""
utils_mg400.py — Shared helpers for DOBOT MG400 scripts (ME403).

Typical imports: connect, clamp, safe_move, safe_rel_move, go_home, parse_pose,
parse_angles, check_errors, start_feedback_thread, MG400_IP, READY_POSE, SAFE_BOUNDS,
current_pose.

Resolves dobot_api.py from mg400/ or ME403_LabFiles/vendor/TCP-IP-4Axis-Python.
Network, robot IPs, joint ranges, and SDK fetch: see robots/mg400/README.md and package docs.
"""

from __future__ import annotations

from argparse import ArgumentParser, Namespace
import os
import re
import socket
import sys
import threading
import time
from contextlib import closing
from pathlib import Path

# ---------------------------------------------------------------------------
# SDK auto-discovery
# ---------------------------------------------------------------------------

_HERE = Path(__file__).parent
_PACKAGE_ROOT = _HERE.parents[1]
_VENDOR_PATH = _PACKAGE_ROOT / "vendor" / "TCP-IP-4Axis-Python"


def _find_dobot_api() -> str:
    """Return the directory containing dobot_api.py, or raise ImportError."""
    if (_HERE / "dobot_api.py").exists():
        return str(_HERE)
    for base in (_PACKAGE_ROOT, *_HERE.parents):
        candidate = base / "vendor" / "TCP-IP-4Axis-Python"
        if (candidate / "dobot_api.py").exists():
            return str(candidate)
    raise ImportError(
        "dobot_api.py not found.\n"
        "Clone the official SDK with:\n"
        "  git clone https://github.com/Dobot-Arm/TCP-IP-4Axis-Python.git "
        f"{_VENDOR_PATH}\n"
        "Or place dobot_api.py directly in mg400/."
    )


_SIM_DIR = _PACKAGE_ROOT / "simulation" / "runtime"

# Eagerly add the SDK directory to sys.path when available (no import yet).
# This preserves backward compatibility for scripts such as 12_feedback_monitor.py
# that do 'from dobot_api import X' directly after importing utils_mg400.
# Simulation mode (DOBOT_SIMULATION=1) works even when the SDK is absent.
try:
    _api_dir = _find_dobot_api()
    if _api_dir not in sys.path:
        sys.path.insert(0, _api_dir)
except ImportError:
    pass


def _simulation_requested() -> bool:
    return os.environ.get("DOBOT_SIMULATION") == "1"


def _ensure_simulation_path() -> None:
    for base in (_SIM_DIR, *(parent / "simulation" / "runtime" for parent in _HERE.parents)):
        if base.exists() and str(base) not in sys.path:
            sys.path.insert(0, str(base))
            return


def _load_dobot_api():
    """Lazily load SDK classes. Raises ImportError if SDK is absent."""
    api_dir = _find_dobot_api()
    if api_dir not in sys.path:
        sys.path.insert(0, api_dir)
    from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType  # noqa: E402
    return DobotApiDashboard, DobotApiMove, DobotApi, MyType

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ROBOT_IPS = {
    1: "192.168.2.7",
    2: "192.168.2.10",
    3: "192.168.2.9",
    4: "192.168.2.6",
}
ROBOT_IDS      = tuple(ROBOT_IPS)
MG400_IP_ENV   = "DOBOT_MG400_IP"
MG400_IP       = os.environ.get(MG400_IP_ENV, ROBOT_IPS[1])   # default: Robot 1, env-overridable
PC_STATIC_IP   = "192.168.2.100"
PC_PREFIX_LEN  = 24
DASHBOARD_PORT = 29999
MOVE_PORT      = 30003
FEED_PORT      = 30004
REQUIRED_PORTS = (DASHBOARD_PORT, MOVE_PORT, FEED_PORT)

# Operating safe bounds (inside the 440 mm hardware envelope).
# Z is non-negative: 0 mm = mounting surface, max 150 mm above.
SAFE_BOUNDS = {
    "x": (60,   400),   # inner: base singularity ≥60 mm; outer: ≤440 mm reach
    "y": (-220, 220),   # full lateral sweep
    "z": (5,    140),   # 5 mm above surface; 10 mm below firmware ceiling
    "r": (-170, 170),   # 10° safety margin inside J4 hardware limit of ±180°
}

# Narrower bounds for in-class demos — stays comfortably inside workspace
CONSERVATIVE_BOUNDS = {
    "x": (150,  350),
    "y": (-150, 150),
    "z": (20,   100),
    "r": (-90,   90),
}

READY_POSE    = (300, 0, 50, 0)   # (x mm, y mm, z mm, r deg) — safe home
JUMP_HEIGHT   = 50                 # mm Z lift above pick/place Z for arch moves
SPEED_DEFAULT = 30                 # % (1–100) — conservative for teaching

# Robot mode values returned by dashboard.RobotMode()
ROBOT_MODE = {
    1:  "INIT",
    2:  "BRAKE_OPEN",
    3:  "POWER_STATUS",
    4:  "DISABLED",
    5:  "ENABLE",        # idle, ready for commands
    6:  "BACKDRIVE",     # drag-teach mode
    7:  "RUNNING",       # executing motion
    8:  "RECORDING",
    9:  "ERROR",         # any uncleared alarm → returns 9 regardless of other state
    10: "PAUSE",
    11: "JOG",
}

# Common MG400 controller alarm codes (from alarm_controller.json).
# Covers the errors most likely encountered in a lab setting.
MG400_ALARM = {
    16: ("Inverse kinematics singularity",
         "Reselect movement points away from singular configurations."),
    17: ("Inverse kinematics has no solution",
         "Reselect movement points inside the reachable workspace."),
    18: ("Inverse kinematics result outside working area",
         "Reselect movement points; the Cartesian pose requires a joint outside its limit."),
    32: ("Inverse kinematics singularity during motion",
         "Reselect movement points away from singular configurations."),
    33: ("Inverse kinematics has no solution during motion",
         "Reselect movement points inside the reachable workspace."),
    34: ("Inverse kinematics result outside working area during motion",
         "Reselect movement points; the path requires a joint outside its limit."),
    85: ("E-stop triggered",
         "Release the hardware E-stop button (twist to unlock), "
         "then run the script again. No power-cycle needed."),
    1:  ("Joint 1 overload", "Check for obstruction and power-cycle the robot."),
    2:  ("Joint 2 overload", "Check for obstruction and power-cycle the robot."),
    3:  ("Joint 3 overload", "Check for obstruction and power-cycle the robot."),
    4:  ("Joint 4 overload", "Check for obstruction and power-cycle the robot."),
    11: ("Joint 1 positive soft-limit", "Move joint back inside limits."),
    12: ("Joint 1 negative soft-limit", "Move joint back inside limits."),
}

# ---------------------------------------------------------------------------
# Shared feedback state (updated by start_feedback_thread)
# ---------------------------------------------------------------------------

current_pose: dict = {"x": 0.0, "y": 0.0, "z": 0.0, "r": 0.0, "valid": False}
_feedback_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------


def connect(ip: str = MG400_IP):
    """Connect to MG400. Returns (dashboard, move_api, feed).

    Call EnableRobot() on dashboard before any motion commands.
    Call DisableRobot() + close all three objects in the finally block.

    The feed object is a raw DobotApi socket to port 30004.
    Read 1440-byte packets and parse with numpy MyType dtype (see
    start_feedback_thread and 12_feedback_monitor.py for usage).
    """
    if _simulation_requested():
        _ensure_simulation_path()
        from sim_mg400 import SimMG400  # noqa: E402
        backend = os.environ.get("DOBOT_SIM_BACKEND", "mujoco")
        sim = SimMG400(backend=backend)
        return sim.connect()
    DobotApiDashboard, DobotApiMove, DobotApi, _ = _load_dobot_api()
    dashboard = DobotApiDashboard(ip, DASHBOARD_PORT)
    move_api  = DobotApiMove(ip, MOVE_PORT)
    feed      = DobotApi(ip, FEED_PORT)
    return dashboard, move_api, feed


def resolve_target_ip(ip: str | None = None, robot: int | None = None) -> str:
    """Return the selected MG400 target IP from robot, explicit IP, or env."""
    if robot is not None:
        return ROBOT_IPS[robot]
    return ip or os.environ.get(MG400_IP_ENV, ROBOT_IPS[1])


def add_target_arguments(
    parser: ArgumentParser,
    *,
    default_ip: str = MG400_IP,
    default_robot: int | None = None,
    ip_help: str = "MG400 IP address",
    robot_help: str = "Robot number 1-4 (overrides --ip)",
) -> None:
    """Attach the standard MG400 `--ip/--robot` arguments to a parser."""
    parser.add_argument("--ip", default=default_ip, help=ip_help)
    parser.add_argument(
        "--robot",
        type=int,
        default=default_robot,
        choices=ROBOT_IDS,
        metavar="N",
        help=robot_help,
    )


def format_direct_connect_help(ip: str, platform_name: str | None = None) -> str:
    """Return a concise setup hint for direct MG400 Ethernet connections."""
    platform_name = platform_name or sys.platform
    lines = [
        "Direct connection checklist:",
        "  1. Power the robot and connect Ethernet directly to the PC",
        f"  2. Set the PC adapter IPv4 to {PC_STATIC_IP}/{PC_PREFIX_LEN} (255.255.255.0)",
        f"  3. Verify with: ping {ip}",
        f"  4. Use --ip {ip} or set {MG400_IP_ENV}",
    ]
    return "\n".join(lines + _platform_direct_connect_lines(platform_name))


def _platform_direct_connect_lines(platform_name: str) -> list[str]:
    """Return OS-specific direct-connect guidance with lazy Windows imports."""
    if platform_name == "win32":
        return _load_windows_direct_connect_lines()
    return [
        "Linux/macOS:",
        "  Use Network Manager or nmcli to set the adapter to 192.168.2.100/24.",
    ]


def _load_windows_direct_connect_lines() -> list[str]:
    """Load Windows-only MG400 guidance lazily."""
    try:
        from windows.mg400_support import get_direct_connect_help_lines
    except Exception:
        return [
            "Windows PowerShell (from repo root):",
            "  .\\windows\\Set-MG400StaticIp.ps1   # dry run only",
            "  Get-NetAdapter",
            "  Run PowerShell as Administrator for the -Apply step",
            "  .\\windows\\Set-MG400StaticIp.ps1 -InterfaceAlias '<EthernetName>' -Apply",
            "  The PowerShell helper does not change the adapter unless you add -Apply.",
        ]
    return get_direct_connect_help_lines()


def _probe_required_ports(ip: str, ports: tuple[int, ...] = REQUIRED_PORTS, timeout: float = 1.5) -> None:
    """Fail fast with a normal socket error before the vendor SDK prints noise."""
    for port in ports:
        try:
            with closing(socket.create_connection((ip, port), timeout=timeout)):
                pass
        except OSError as exc:
            raise OSError(f"TCP port {port} is unreachable: {exc}") from exc


def _connection_error(ip: str, message: str) -> ConnectionError:
    """Build a consistent connection error with setup guidance."""
    return ConnectionError(f"{message}\n{format_direct_connect_help(ip)}")


def connect_with_diagnostics(ip: str = MG400_IP):
    """Connect to MG400 or raise ConnectionError with setup guidance."""
    if _simulation_requested():
        return connect(ip)
    try:
        _probe_required_ports(ip)
    except OSError as exc:
        raise _connection_error(ip, f"Cannot reach MG400 at {ip}: {exc}") from exc

    try:
        return connect(ip)
    except ConnectionRefusedError as exc:
        raise _connection_error(ip, f"Connection refused for MG400 at {ip}.") from exc
    except OSError as exc:
        raise _connection_error(ip, f"Cannot connect to MG400 at {ip}: {exc}") from exc
    except Exception as exc:
        raise _connection_error(ip, f"MG400 connection setup failed at {ip}: {exc}") from exc


def resolve_target_from_args(args: Namespace) -> str:
    """Resolve the MG400 target IP from parsed CLI arguments."""
    return resolve_target_ip(ip=getattr(args, "ip", None), robot=getattr(args, "robot", None))


def connect_from_args_or_exit(args: Namespace):
    """Resolve CLI target args, connect, or exit with a clean script error."""
    ip = resolve_target_from_args(args)
    try:
        dashboard, move_api, feed = connect_with_diagnostics(ip)
    except ConnectionError as exc:
        raise SystemExit(f"[Error] {exc}") from exc
    return ip, dashboard, move_api, feed


def close_all(dashboard, move_api, feed) -> None:
    """Close all three API connections safely."""
    for obj in (dashboard, move_api, feed):
        try:
            obj.close()
        except Exception:
            pass


def connect_multi(robot_ids=None):
    """Connect to multiple MG400 robots simultaneously.

    Args:
        robot_ids: list of ints 1-4. None = all 4 robots.

    Returns:
        dict mapping robot_id → (dashboard, move_api, feed)
    """
    if robot_ids is None:
        robot_ids = list(ROBOT_IPS.keys())
    return {rid: connect(ROBOT_IPS[rid]) for rid in robot_ids}


def close_all_robots(robots: dict) -> None:
    """Close all connections returned by connect_multi()."""
    for _rid, (dashboard, move_api, feed) in robots.items():
        close_all(dashboard, move_api, feed)

# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------


def query_dashboard_version(dashboard) -> str:
    """Query firmware version across SDK variants."""
    if hasattr(dashboard, "GetVersion"):
        response = dashboard.GetVersion()
    elif hasattr(dashboard, "sendRecvMsg"):
        response = dashboard.sendRecvMsg("GetVersion()")
    else:
        raise AttributeError("dashboard object has neither GetVersion nor sendRecvMsg")

    match = re.match(r"\s*([-+]?\d+),", str(response))
    if match and int(match.group(1)) != 0:
        raise ValueError(f"GetVersion returned status {match.group(1)}")
    return response


def parse_pose(response: str) -> tuple[float, float, float, float]:
    """Parse GetPose() response string → (x, y, z, r).

    Expected format: "0,{x},{y},{z},{rx},{ry},{rz},{r}" or "0,{x},{y},{z},{r}"
    The leading '0' is the error code; rx/ry are always 0 for the 4-axis MG400.
    """
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", response)
    floats = [float(n) for n in nums]
    if len(floats) >= 5:
        # SDK responses include return code first: 0 means success.
        status = int(floats[0])
        if status != 0:
            raise ValueError(f"GetPose returned status {status}: {response!r}")
        # Skip return code; take x,y,z,r (indices 1,2,3,4)
        return floats[1], floats[2], floats[3], floats[4]
    if len(floats) == 4:
        # Legacy/fallback format without a leading status code.
        return floats[0], floats[1], floats[2], floats[3]
    raise ValueError(f"Cannot parse pose from: {response!r}")


def parse_angles(response: str) -> tuple[float, float, float, float]:
    """Parse GetAngle() response string → (j1, j2, j3, j4).

    Expected format: "0,{j1},{j2},{j3},{j4}" (degrees).
    """
    nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", response)
    floats = [float(n) for n in nums]
    if len(floats) >= 5:
        status = int(floats[0])
        if status != 0:
            raise ValueError(f"GetAngle returned status {status}: {response!r}")
        return floats[1], floats[2], floats[3], floats[4]
    if len(floats) == 4:
        # Legacy/fallback format without a leading status code.
        return floats[0], floats[1], floats[2], floats[3]
    raise ValueError(f"Cannot parse angles from: {response!r}")


def parse_robot_mode(response: str) -> int:
    """Parse RobotMode() response → integer mode code."""
    nums = [int(n) for n in re.findall(r"[-+]?\d+", response)]
    if not nums:
        raise ValueError(f"Cannot parse robot mode from: {response!r}")

    if len(nums) >= 2 and nums[0] in (0, -1):
        status = nums[0]
        if status != 0:
            raise ValueError(f"RobotMode returned status {status}: {response!r}")
        mode = nums[1]
    elif len(nums) == 1:
        # Fallback for response styles that return mode directly.
        mode = nums[0]
    else:
        mode = nums[-1]

    if mode not in ROBOT_MODE:
        raise ValueError(f"RobotMode payload missing/invalid in: {response!r}")
    return mode


def parse_error_ids(response: str) -> list[int]:
    """Parse GetErrorID() response → list[int] of non-zero error IDs.

    Valid empty-alarm payloads still include a bracketed matrix, e.g.
    "0,{[[],[],...]} ,GetErrorID();". Echo-only payloads like "GetErrorID()"
    or "0,{},GetErrorID();" are treated as malformed.
    """
    text = (response or "").strip()
    if not text:
        raise ValueError("Empty GetErrorID response")
    if "[" not in text or "]" not in text:
        raise ValueError(f"Malformed GetErrorID payload (missing matrix): {response!r}")

    nums = [int(n) for n in re.findall(r"[-+]?\d+", text)]
    if not nums:
        raise ValueError(f"Cannot parse GetErrorID response: {response!r}")

    status = nums[0]
    if status != 0:
        raise ValueError(f"GetErrorID returned status {status}: {response!r}")

    return [n for n in nums[1:] if n != 0]

# ---------------------------------------------------------------------------
# Safety helpers
# ---------------------------------------------------------------------------


def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp v to [lo, hi]."""
    return max(lo, min(hi, v))


def safe_move(
    move_api,
    x: float, y: float, z: float, r: float,
    mode: str = "J",
    bounds: dict | None = None,
) -> None:
    """Clamp (x,y,z,r) to bounds and send MovJ or MovL.

    Does NOT call Sync() — the caller decides when to synchronise.
    Prints a warning whenever any axis is clamped.

    Args:
        move_api: DobotApiMove instance
        x,y,z,r:  target pose (mm and degrees)
        mode:      "J" → MovJ (joint interpolation, default)
                   "L" → MovL (straight-line Cartesian path)
        bounds:    dict with keys "x","y","z","r" each a (lo,hi) tuple.
                   Defaults to SAFE_BOUNDS.
    """
    b  = bounds or SAFE_BOUNDS
    cx = clamp(x, *b["x"])
    cy = clamp(y, *b["y"])
    cz = clamp(z, *b["z"])
    cr = clamp(r, *b["r"])
    if (cx, cy, cz, cr) != (x, y, z, r):
        print(
            f"[safe_move] Clamped: ({x:.1f},{y:.1f},{z:.1f},{r:.1f})"
            f" -> ({cx:.1f},{cy:.1f},{cz:.1f},{cr:.1f})"
        )
    if mode.upper() == "L":
        move_api.MovL(cx, cy, cz, cr)
    else:
        move_api.MovJ(cx, cy, cz, cr)


def safe_rel_move(
    move_api,
    dashboard,
    dx: float = 0, dy: float = 0, dz: float = 0, dr: float = 0,
    mode: str = "J",
) -> None:
    """Move relative to current pose, clamped to SAFE_BOUNDS.

    Reads the current pose via dashboard.GetPose(), adds the deltas,
    then delegates to safe_move(). All clamping warnings apply.
    """
    resp = dashboard.GetPose()
    x, y, z, r = parse_pose(resp)
    safe_move(move_api, x + dx, y + dy, z + dz, r + dr, mode=mode)


def go_home(move_api) -> None:
    """Move to READY_POSE (joint interpolation) and Sync."""
    move_api.MovJ(*READY_POSE)
    move_api.Sync()
    print(f"[utils] At home: {READY_POSE}")

# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------


def check_errors(dashboard) -> None:
    """Query GetErrorID(), print any active errors, then ClearError + Continue.

    Safe to call at startup; silently returns if no errors exist.
    Raises RuntimeError if errors cannot be cleared (robot needs physical
    intervention — check E-stop, collision guards, joint limits, then power-cycle).
    """
    try:
        resp = dashboard.GetErrorID()
        error_ids = parse_error_ids(resp)
        if error_ids:
            for eid in error_ids:
                name, fix = MG400_ALARM.get(eid, ("Unknown alarm", "Power-cycle the robot."))
                print(f"[check_errors] Error {eid}: {name}")
                print(f"               Fix: {fix}")
            dashboard.ClearError()
            cont_resp = dashboard.Continue()
            # A -1 return code means the command was rejected (robot still in ERROR)
            if not cont_resp or cont_resp.strip().startswith("-1"):
                descriptions = "; ".join(
                    MG400_ALARM.get(e, ("Unknown alarm", ""))[0] for e in error_ids
                )
                raise RuntimeError(
                    f"[check_errors] Cannot clear robot errors {error_ids} ({descriptions}).\n"
                    "  Robot is still in ERROR mode — do not proceed.\n"
                    "  See fix instructions printed above."
                )
            print("[check_errors] Errors cleared, robot resumed.")
        else:
            print("[check_errors] No errors.")
    except RuntimeError:
        raise
    except Exception as exc:
        print(f"[check_errors] Could not read errors: {exc}")

# ---------------------------------------------------------------------------
# Feedback thread
# ---------------------------------------------------------------------------


def start_feedback_thread(feed) -> threading.Thread:
    """Spawn a daemon thread that reads port 30004 binary packets at 8 ms.

    Parses each 1440-byte packet using the SDK's MyType numpy dtype and
    updates the shared current_pose dict with tool_vector_actual (x,y,z,r).

    Updates the shared current_pose dict continuously.
    Thread is a daemon so it exits when the main program exits.

    Args:
        feed: DobotApi instance connected to port 30004 (from connect()).

    Returns:
        The running Thread object (daemon; no need to join).
    """
    if not hasattr(feed, "socket_dobot"):
        # Simulation feed stub has no socket; return a harmless no-op thread.
        t = threading.Thread(target=lambda: None, daemon=True)
        t.start()
        return t

    from dobot_api import MyType  # noqa: E402 — lazy; SDK already on sys.path
    import numpy as np

    PACKET_SIZE = 1440
    TEST_VALUE  = 0x123456789ABCDEF

    def _read() -> None:
        while True:
            try:
                data    = bytes()
                has_read = 0
                while has_read < PACKET_SIZE:
                    chunk = feed.socket_dobot.recv(PACKET_SIZE - has_read)
                    if chunk:
                        has_read += len(chunk)
                        data     += chunk
                info = np.frombuffer(data, dtype=MyType)
                if int(info["test_value"][0]) == TEST_VALUE:
                    tv = info["tool_vector_actual"][0]   # [x, y, z, r, ...]
                    with _feedback_lock:
                        current_pose["x"]     = float(tv[0])
                        current_pose["y"]     = float(tv[1])
                        current_pose["z"]     = float(tv[2])
                        current_pose["r"]     = float(tv[3])
                        current_pose["valid"] = True
            except Exception:
                pass
            time.sleep(0.001)

    t = threading.Thread(target=_read, daemon=True)
    t.start()
    return t


def wait_arrive(
    target: tuple[float, float, float],
    tol_mm: float = 2.0,
    timeout: float = 20.0,
) -> bool:
    """Block until current_pose is within tol_mm of target (x,y,z).

    Requires start_feedback_thread() to be running.

    Returns:
        True if arrived within timeout, False on timeout.
    """
    tx, ty, tz = target[0], target[1], target[2]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with _feedback_lock:
            valid = current_pose["valid"]
            px, py, pz = current_pose["x"], current_pose["y"], current_pose["z"]
        if valid:
            dist = ((px - tx) ** 2 + (py - ty) ** 2 + (pz - tz) ** 2) ** 0.5
            if dist < tol_mm:
                return True
        time.sleep(0.05)
    print(f"[wait_arrive] Timeout: could not reach {target} within {timeout:.0f}s")
    return False
