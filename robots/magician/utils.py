"""
Shared helpers for Dobot Magician scripts (ME403).

Typical imports: clamp, safe_move, safe_rel_move, go_home, unpack_pose,
prepare_robot, check_alarms, find_port, HOME_JOINTS, SAFE_READY_POSE, SAFE_BOUNDS,
MAGICIAN_TOOL_OFFSETS, set_tool_offset, get_tool_offset, JUMP_HEIGHT, SPEED_DEFAULT.

On import, _patch_pydobotplus() patches move_to to skip an extra get_pose round-trip
and a noisy print when x,y,z are all given. See magician/README.md for hardware setup.
"""

import os
import struct
import sys
import time

from serial.tools import list_ports

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

HOME_JOINTS = (0, 0, 0, 0)  # J1, J2, J3, J4 (deg) — joint-space home
SAFE_READY_POSE = (200, 0, 100, 0)  # X, Y, Z (mm), R (deg) — Cartesian staging pose
READY_POSE = SAFE_READY_POSE  # backward compat

# Physical/firmware hard limits — used only for visualization reference.
# Do NOT use these as motion targets; they are the boundaries the firmware
# enforces (or where joint singularities/cable-wrap risks begin).
HARD_LIMITS = {
    "x": (115, 320),   # full Cartesian reach envelope
    "y": (-160, 160),  # arm geometry limit
    "z": (0, 160),     # 0 mm = table surface; 160 mm = firmware ceiling
    "r": (-135, 135),  # servo range (cable-wrap risk past ±90°)
}

# Operating safe bounds — minimal clearance from hard limits.
# All motion commands are clamped here; safe_move() will warn when clamping.
SAFE_BOUNDS = {
    "x": (120, 315),   # was (150,280) — 5 mm from base singularity / max reach
    "y": (-158, 158),  # was (-160,160) — 2 mm margin
    "z": (5, 155),     # was (10,150) — 5 mm above table, 5 mm below ceiling
    "r": (-90, 90),    # keep ±90° to avoid cable wrap despite servo ±135° range
}

# Tighter bounds for demos — stays well inside reachable workspace to avoid
# POSE_LIMIT_OVER and joint limits. Use when SAFE_BOUNDS targets hit limits.
CONSERVATIVE_BOUNDS = {
    "x": (170, 250),
    "y": (-120, 120),
    "z": (30, 120),
    "r": (-60, 60),
}

# Speed profiles (velocity mm/s, acceleration mm/s²)
SAFE_VELOCITY     = 100   # mm/s  (~33 % of max)
SAFE_ACCELERATION = 80    # mm/s²
SPEED_DEFAULT    = (SAFE_VELOCITY, SAFE_ACCELERATION)
SPEED_SMOOTH     = (50, 40)   # gentler for demos

JUMP_HEIGHT = 30  # mm — default Z clearance for JUMP_XYZ mode

# ---------------------------------------------------------------------------
# Tool / end-effector offset API
# ---------------------------------------------------------------------------

# Named presets matching DobotStudio factory tool-coordinate selections.
# Each tuple is (ox_mm, oy_mm, oz_mm) applied along the tool X/Y/Z axes.
MAGICIAN_TOOL_OFFSETS: dict[str, tuple[float, float, float]] = {
    "none":    (0.0,  0.0, 0.0),   # bare flange          → home TCP (147, 0, 135) mm
    "motor":   (60.0, 0.0, 0.0),   # motor flange          → home TCP (207, 0, 135) mm
    "suction": (60.0, 0.0, -70.0),  # suction cup  → home TCP (207, 0, 65) mm (physical cup tip)
                                     # NOTE: DobotStudio "suction cup" factory preset = (+60,0,0),
                                     # same as motor — it reports the motor-shaft TCP, not the cup tip.
                                     # Use "motor" preset to match DobotStudio GetPose readings.
}


def set_tool_offset(bot, ox: float, oy: float, oz: float) -> None:
    """Write TCP offset (mm) to Magician firmware (protocol 60).

    Same idea as DobotStudio tool presets; use MAGICIAN_TOOL_OFFSETS for named offsets.
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
    """Read current TCP offset (ox, oy, oz) in mm from Magician firmware."""
    from pydobotplus.dobotplus import Message
    msg = Message()
    msg.id = 60
    msg.ctrl = 0x00
    resp = bot._send_command(msg)
    ox = struct.unpack_from('f', resp.params, 0)[0]
    oy = struct.unpack_from('f', resp.params, 4)[0]
    oz = struct.unpack_from('f', resp.params, 8)[0]
    return float(ox), float(oy), float(oz)


# ---------------------------------------------------------------------------
# Port discovery
# ---------------------------------------------------------------------------

# Dobot Magician uses either CP210x (Silicon Labs) or CH340 (1A86) USB-serial chips
DOBOT_KEYWORDS = ("Silicon Labs", "1A86", "USB2.0-Serial")
DOBOT_PORT_ENV = "DOBOT_PORT"


def find_port(keywords: tuple[str, ...] = DOBOT_KEYWORDS) -> str | None:
    """First port matching keywords in description/hwid, or DOBOT_PORT if set, or best USB fallback."""
    preferred = os.environ.get(DOBOT_PORT_ENV)
    if preferred:
        return preferred

    ports = list(list_ports.comports())
    def desc_hwid(port) -> str:
        return f"{(port.description or '')} {port.hwid}".lower()

    for p in ports:
        combined = desc_hwid(p)
        if any(kw.lower() in combined for kw in keywords):
            return p.device
    fallback = _select_fallback_port(ports)
    return fallback.device if fallback else None


def _select_fallback_port(ports, platform_name: str | None = None):
    """Return the best fallback serial port for the active platform."""
    platform_name = platform_name or sys.platform
    if not ports:
        return None

    def desc_hwid(port) -> str:
        return f"{(port.description or '')} {port.hwid}".lower()

    def score(port) -> tuple[int, str]:
        device = (port.device or "")
        device_l = device.lower()
        combined = desc_hwid(port)
        value = 0

        if getattr(port, "vid", None) is not None or "usb vid:pid" in combined:
            value += 20
        if any(token in combined for token in ("usb", "uart", "serial", "cp210", "ch340", "silicon labs", "wch")):
            value += 15
        if "bluetooth" in combined or "virtual" in combined:
            value -= 20

        if platform_name == "win32":
            if device.upper().startswith("COM"):
                value += 25
        elif platform_name == "darwin":
            if "/dev/cu." in device_l or "/dev/tty.usb" in device_l:
                value += 25
        else:
            if "/ttyusb" in device_l or "/ttyacm" in device_l:
                value += 25
            if "/ttys" in device_l:
                value -= 20

        return value, device

    return max(ports, key=score)


# ---------------------------------------------------------------------------
# Safety helpers
# ---------------------------------------------------------------------------

def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp *v* to the closed interval [lo, hi]."""
    return max(lo, min(hi, v))


def unpack_pose(pose) -> tuple[float, float, float, float, float, float, float, float]:
    """Flat 8-tuple x,y,z,r,j1,j2,j3,j4 from pydobotplus Pose or an 8-number sequence."""
    if hasattr(pose, "position") and hasattr(pose, "joints"):
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.position.r),
            float(pose.joints.j1),
            float(pose.joints.j2),
            float(pose.joints.j3),
            float(pose.joints.j4),
        )

    if isinstance(pose, (tuple, list)) and len(pose) == 8:
        return tuple(float(v) for v in pose)

    raise ValueError(f"Unsupported pose format: {type(pose)!r}")


def safe_move(bot, x: float, y: float, z: float, r: float, mode=None,
              bounds: dict | None = None, verify: bool = False,
              verify_tol_mm: float = 5, verify_tol_deg: float = 5) -> None:
    """Cartesian move with clamping (default SAFE_BOUNDS; optional CONSERVATIVE_BOUNDS).

    Optional verify compares achieved pose; mode selects PTP mode when set.
    """
    b = bounds or SAFE_BOUNDS
    cx = clamp(x, *b["x"])
    cy = clamp(y, *b["y"])
    cz = clamp(z, *b["z"])
    cr = clamp(r, *b["r"])
    if (cx, cy, cz, cr) != (x, y, z, r):
        print(f"[safe_move] Clamped: ({x:.1f},{y:.1f},{z:.1f},{r:.1f})"
              f" -> ({cx:.1f},{cy:.1f},{cz:.1f},{cr:.1f})")
    if mode is not None:
        bot.move_to(cx, cy, cz, cr, wait=True, mode=mode)
    else:
        bot.move_to(cx, cy, cz, cr, wait=True)
    if verify:
        ax, ay, az, ar, *_ = unpack_pose(bot.get_pose())
        dx = abs(ax - cx)
        dy = abs(ay - cy)
        dz = abs(az - cz)
        dr = abs(ar - cr)
        if dx > verify_tol_mm or dy > verify_tol_mm or dz > verify_tol_mm or dr > verify_tol_deg:
            print(f"[safe_move] LIMIT: target ({cx:.1f},{cy:.1f},{cz:.1f},{cr:.1f}) "
                  f"-> achieved ({ax:.1f},{ay:.1f},{az:.1f},{ar:.1f}) "
                  f"(drift: dx={dx:.1f} dy={dy:.1f} dz={dz:.1f} dr={dr:.1f})")


def go_home(bot) -> None:
    """Send *bot* to joint-space home (0, 0, 0, 0)."""
    from pydobotplus.dobotplus import MODE_PTP
    bot.move_to(*HOME_JOINTS, wait=True, mode=MODE_PTP.MOVJ_ANGLE)
    print("[utils] At home: joint zero (0, 0, 0, 0)")


def safe_rel_move(bot, dx: float = 0, dy: float = 0, dz: float = 0, dr: float = 0) -> None:
    """Move relative to the current pose, clamped to SAFE_BOUNDS.

    Reads the current pose, adds the deltas, then delegates to safe_move().
    All clamping and warning logic from safe_move() applies.
    """
    x, y, z, r, *_ = unpack_pose(bot.get_pose())
    safe_move(bot, x + dx, y + dy, z + dz, r + dr)


def check_alarms(bot) -> None:
    """Warn if the robot has active alarms; clears them. Call after connecting.

    Prints each alarm's name so students can identify the fault quickly.
    If no alarms are active, returns silently.
    """
    alarms = bot.get_alarms()
    if alarms:
        print(f"[check_alarms] WARNING: {len(alarms)} alarm(s) detected:")
        for a in alarms:
            print(f"  {a.name}")
        bot.clear_alarms()
        print("[check_alarms] Alarms cleared.")


def do_homing(bot) -> None:
    """Run the robot homing sequence. Call after power-on or when LIMIT_* alarms occur.

    Power-on position is NOT home. The robot must be homed to establish its
    coordinate frame before motion. Homing moves the arm to physical limit
    switches and calibrates encoders. Takes ~15-30 seconds.
    """
    print("[do_homing] Running homing sequence (15-30 s) ...")
    bot.home()
    time.sleep(25)  # homing takes 15-30 s; home() may not block
    print("[do_homing] Homing complete.")


def prepare_robot(bot) -> None:
    """Clear alarms; run homing if LIMIT alarms were present. Call right after connect."""
    print(
        "[prepare_robot] NOTE: Calibrate/home in original Dobot software "
        "(DobotStudio or DobotLab) before Python runs."
    )
    print("[prepare_robot] NOTE: Dobot software and Python cannot control the robot simultaneously.")
    alarms = bot.get_alarms()
    if alarms:
        alarm_names = [str(getattr(a, "name", a)) for a in alarms]
        print(f"[prepare_robot] Clearing {len(alarms)} alarm(s):")
        for name in alarm_names:
            print(f"  {name}")
        bot.clear_alarms()
        if any("LIMIT" in name.upper() for name in alarm_names):
            do_homing(bot)


def startup_check(bot) -> None:
    """Alias for prepare_robot. Clear alarms and run homing if LIMIT alarms present."""
    prepare_robot(bot)


# ---------------------------------------------------------------------------
# pydobotplus compatibility patch (applied automatically on import)
# ---------------------------------------------------------------------------

def _patch_pydobotplus() -> None:
    """Faster move_to when x,y,z are set: skip extra get_pose and spam print. Partial calls unchanged.

    Applied at import; no-op if pydobotplus layout changes.
    """
    try:
        import pydobotplus.dobotplus as _dp
        from pydobotplus.dobotplus import MODE_PTP

        _orig = _dp.Dobot.move_to

        def _fast_move_to(self, x=None, y=None, z=None, r=0,
                          wait=True, mode=None, position=None):
            if position is not None:
                x, y, z, r = position.x, position.y, position.z, position.r
            if x is None or y is None or z is None:
                # Partial call: fall back to original (needs get_pose for Nones)
                return _orig(self, x=x, y=y, z=z, r=r, wait=wait, mode=mode)
            if mode is None:
                mode = MODE_PTP.MOVJ_XYZ
            return self._extract_cmd_index(
                self._set_ptp_cmd(x, y, z, r, mode, wait=wait)
            )

        _dp.Dobot.move_to = _fast_move_to
    except Exception:
        pass  # graceful degradation if pydobotplus internals change


_patch_pydobotplus()
