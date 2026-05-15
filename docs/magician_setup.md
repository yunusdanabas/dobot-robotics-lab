# Dobot Magician Setup

Hardware and software setup for the **Dobot Magician** (USB serial) when using this course package.

The Magician uses USB serial (CP210x Silicon Labs chip, 115200 baud).

Python snippets use `import utils as U` from **your working directory** (typically an exercise folder under `labs/`, or `robots/magician/`, with that directory on `PYTHONPATH`). API details: [`student_api.md`](student_api.md).

## Setup Checklist

1. Connect USB and power.
2. Install DobotStudio (DOBOT Magician) v1.9.4.
3. Connect in DobotStudio and run **Home/Calibration** once before Python.
4. In DobotStudio end-effector settings, select end-effector type `None`.
5. In the advanced end-effector menu, set XYZ bias to `0, 0, 0`.
6. **Disconnect/close** DobotStudio before running Python scripts.
7. Only one controller can own the robot at a time.
8. On Linux, add your user to `dialout` once:

```bash
sudo usermod -a -G dialout $USER
# then log out and back in
```

Check the port from `ME403_LabFiles/`:

```bash
python3 scripts/check_magician.py
```

## End-Effector Modes

The Magician stores a TCP offset (tool bias) in firmware. This offset shifts
all `GetPose` positions by `(ox, oy, oz)` mm relative to the bare flange.

| Mode | TCP offset (ox, oy, oz) mm | Home position (0,0,0,0) |
|------|---------------------------|------------------------|
| `none` (bare flange) | (0, 0, 0) | (147, 0, 135) mm |
| `motor` (motor flange) | (60, 0, 0) | (207, 0, 135) mm |
| `suction` (physical cup tip) | (60, 0, -70) | (207, 0, 65) mm |

**Note:** DobotStudio's factory "suction cup" preset uses `(+60, 0, 0)` — the
same as motor (motor-shaft TCP, not the physical cup tip). Use the `motor` preset
when you want `GetPose` to match DobotStudio readings with the suction cup attached.
Use `suction` in simulation/FK to model the physical cup tip. You can verify your
current offset with `get_tool_offset(bot)`.

## Setting End-Effector Type from Python

The TCP offset can also be changed from Python — no DobotStudio session needed. `set_tool_offset()` / `get_tool_offset()` use Protocol ID 60 on the Magician; the course bundle implements them in **`robots/magician/utils.py`** and in exercise **`utils.py`** modules under `labs/` (same call pattern when the first argument is the live `Dobot` instance).

If your helper layer wraps the driver (e.g. session object with a `handles` map), pass the **inner Dobot** that owns the serial connection into `set_tool_offset` / `get_tool_offset`.

```python
import utils as U

bot = U.setup()

# Check what the firmware currently has
ox, oy, oz = U.get_tool_offset(bot)
print(f"Current TCP offset: ({ox:.1f}, {oy:.1f}, {oz:.1f}) mm")

# Switch to bare-flange mode (no end effector)
U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["none"])

# Switch to suction-cup / motor-flange mode
U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["suction"])
```

The setting persists after power-off (stored in EEPROM). For forward-kinematics exercises with a bare flange, use `none` mode so `GetPose` returns positions at the bare wrist axis.

## Port Discovery

`utils.py` in your exercise tree (or `robots/magician/utils.py`) auto-discovers the port by scanning for CP210x/CH340 USB-serial
chips. To override, set `DOBOT_PORT=/dev/ttyUSB0` in the environment.

---

## See also

[`setup.md`](setup.md) · [`simulation.md`](simulation.md) · [`student_api.md`](student_api.md) · [`troubleshooting.md`](troubleshooting.md)
