"""
Minimal Magician helpers for move-homework.

Public API:
    bot = setup()
    x, y, z, r = move_and_get_feedback(bot, q)
    teardown(bot)
"""

import os
from serial.tools import list_ports

JOINT_BOUNDS_FW = {
    "j1": (-90.0, 90.0),
    "j2": (0.0, 85.0),
    "j3": (-10.0, 85.0),
    "j4": (-90.0, 90.0),
}


def find_port():
    """Auto-detect the Dobot serial port."""
    env = os.environ.get("DOBOT_PORT")
    if env:
        return env
    for port in list_ports.comports():
        desc = f"{port.description or ''} {port.hwid}".lower()
        if any(kw in desc for kw in ("silicon labs", "1a86", "usb2.0-serial")):
            return port.device
    ports = list(list_ports.comports())
    return ports[0].device if ports else None


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _to_firmware_angles(q):
    """Body-frame -> firmware angles."""
    return float(q[0]), float(q[1]), float(q[1]+q[2]), float(q[3])


def _clamp_firmware_angles(j1, j2, j3, j4):
    clamped = (
        _clamp(j1, *JOINT_BOUNDS_FW["j1"]),
        _clamp(j2, *JOINT_BOUNDS_FW["j2"]),
        _clamp(j3, *JOINT_BOUNDS_FW["j3"]),
        _clamp(j4, *JOINT_BOUNDS_FW["j4"]),
    )

    return clamped


def _unpack_pose(pose):
    if hasattr(pose, "position"):
        p = pose.position
        return float(p.x), float(p.y), float(p.z), float(p.r)
    return tuple(float(v) for v in pose[:4])


def _go_home(bot):
    from pydobotplus.dobotplus import MODE_PTP

    bot.move_to(0, 0, 0, 0, wait=True, mode=MODE_PTP.MOVJ_ANGLE)


def _move_joint_angles(bot, q):
    from pydobotplus.dobotplus import MODE_PTP

    j1, j2, j3, j4 = _to_firmware_angles(q)
    clamped = _clamp_firmware_angles(j1, j2, j3, j4)
    if clamped != (j1, j2, j3, j4):
        print("Joint values exceed safe limits. Motion denied!")
    else:
        bot.move_to(j1, j2, j3, j4, wait=True, mode=MODE_PTP.MOVJ_ANGLE)


def _read_pose(bot):
    return _unpack_pose(bot.get_pose())


def setup(port=None):
    """Connect, clear alarms, and move to home."""
    from pydobotplus import Dobot

    target = port or find_port()
    if not target:
        raise OSError("No Dobot serial port found. Check USB + power.")
    print(f"[setup] Connecting on {target} ...")
    bot = Dobot(port=target)
    if bot.get_alarms():
        bot.clear_alarms()
    _go_home(bot)
    print("[setup] Ready.")
    return bot


def teardown(bot):
    """Move home and close the connection."""
    try:
        _go_home(bot)
    except Exception:
        pass
    try:
        bot.close()
    except Exception:
        pass
    print("[teardown] Done.")


def move_and_get_feedback(bot, q):
    """Move robot to q=[q1,q2,q3,q4], return the resulting (x, y, z, r)."""
    _move_joint_angles(bot, q)
    return _read_pose(bot)
