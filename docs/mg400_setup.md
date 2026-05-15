# DOBOT MG400 Setup

Hardware and networking for the **DOBOT MG400** (Ethernet TCP/IP) in the course package. API summary: [`student_api.md`](student_api.md).

The MG400 communicates over Ethernet (TCP/IP) — no USB, no serial port.
It exposes three TCP sockets that must all be connected:

| Port | Name | Purpose |
|------|------|---------|
| 29999 | Dashboard | Enable/disable, error query, pose/angle readout, FK/IK queries |
| 30003 | Move API | MovJ, MovL, Arc, RelMovJ, Sync, JointMovJ, MoveJog, … |
| 30004 | Feedback | 1440-byte binary packets at 8 ms (live robot state) |

## Network Setup (one-time per machine)

1. Connect the PC to the robot controller **LAN2** port with an Ethernet cable.
2. Set PC's Ethernet adapter to static IP `192.168.2.100`, netmask `255.255.255.0`.
3. Verify: `ping 192.168.2.7` (or your assigned robot IP).

**Robot IP addresses:**

| Robot | IP |
|-------|----|
| 1 | 192.168.2.7 |
| 2 | 192.168.2.10 |
| 3 | 192.168.2.9 |
| 4 | 192.168.2.6 |

## SDK Setup

The MG400 Python SDK is not on PyPI. Download it from `ME403_LabFiles/`:

```bash
python3 scripts/fetch_assets.py --mg400-sdk
```

This clones `dobot_api.py` into `vendor/TCP-IP-4Axis-Python/`. Packaged teaching **`utils.py`** code discovers it there — no further configuration needed beyond the fetch.

Verify connectivity:

```bash
python3 scripts/check_mg400.py --robot 1
```

## Joint Convention and Coordinate Frame

The MG400 is a 4-DOF parallel-arm robot (440 mm reach, 500 g payload).

| Joint | Firmware zero | Positive direction | Range |
|-------|--------------|-------------------|-------|
| J1 | base aligned with +X | CCW looking down | ±160° |
| J2 | shoulder link horizontal | tilts upward | −25° to +85° |
| J3 | forearm link angle (firmware absolute = J2+J3_body) | extends reach | −25° to +105° |
| J4 | wrist rotation | CCW | ±180° |

**Key constant:** In the MG400 firmware, `J3_firmware = J2_body + J3_body`.
The bundled teaching **`utils.py`** handles this conversion — pass
body-frame angles and read firmware-consistent values from `get_joints()` where provided.

### Geometric FK (MG400)

```
Z_base = 116.0 mm    (mounting surface to shoulder pivot)
L1     = 175.0 mm    (shoulder to elbow)
L2     = 175.0 mm    (elbow to wrist)

R  = L1·cos(J2) + L2·cos(J3_fw - J2)
z  = Z_base + L1·sin(J2) + L2·sin(J3_fw - J2)
x  = R·cos(J1)
y  = R·sin(J1)
```

At firmware `(0, 0, 60, 0)` (MG400 factory home): TCP ≈ `(350, 0, 116)` mm.

## Safe Bounds

| Axis | Safe range | Notes |
|------|-----------|-------|
| X | 60–400 mm | inner limit ≈ 60 mm (base singularity) |
| Y | ±220 mm | full lateral sweep |
| Z | 5–140 mm | **Z cannot go negative** — 0 mm = mounting surface |
| R | ±170° | 10° safety margin inside J4 ±180° |

Always use your stack’s **`safe_move()`** (or equivalent clamped Cartesian helper) when moving in Cartesian space — the bundled MG400 teaching helpers clamp to these limits and warn.

## Ready Pose

```python
READY_POSE = (300, 0, 50, 0)   # safe home above mounting surface
```

## End-Effector Type

ME403 exercise material uses the MG400 at its default bare TCP (no physical attachment).
No tool offset switching is needed — `GetPose` reports the wrist
reference point. Unlike the Magician, there is no `set_tool_offset()` workflow
for the MG400 in this package.

## Simulation Mode

With **`DOBOT_SIMULATION` unset**, packaged teaching code defaults to simulation where that stack is implemented. From your exercise folder (under `labs/`), selecting MG400 is typically done via environment:

```bash
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
# Optional explicit flag (same category of stacks):
DOBOT_SIMULATION=1 DOBOT_ROBOT_TYPE=mg400 python3 interface.py
```

Select robot in Python (set **before** `U.setup()` in that process when your `utils.py` reads this at import):

```python
import os
os.environ["DOBOT_ROBOT_TYPE"] = "mg400"
```

The MuJoCo/PyBullet backends for MG400 are co-equal with Magician backends. See [`simulation.md`](simulation.md) for backend selection and URDF cache details.

---

## See also

[`setup.md`](setup.md) · [`simulation.md`](simulation.md) · [`student_api.md`](student_api.md) · [`troubleshooting.md`](troubleshooting.md)
