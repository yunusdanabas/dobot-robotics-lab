# Lab 01 - API reference (`utils.py`)

Forward kinematics lab - **Dobot Magician or MG400**. Use with `myCode.py`; `interface.py` calls `myCode.run(robot)`.

Full merged reference: [`../../docs/student_api.md`](../../docs/student_api.md).

Run Python **from this folder** so `import utils as U` works (`cd labs/lab01_forward_kinematics`).

MG400 **`setup()` / `teardown()`** move to **firmware joint home `(0,0,0,0)`** (`JointMovJ` + `Sync`) so Task 0 aligns with Magician-style joint home.

---

## Session

`U.setup()` returns **`robot`** (`RobotSession`: `type`, `simulation`, `backend`, `handles`). Use **`U.teardown(robot)`** in a `finally` block.

```python
import utils as U

robot = U.setup()
try:
    # your code in myCode.run(robot)
    ...
finally:
    U.teardown(robot)
```

Optional: `U.configure(robot_type=..., use_simulation=..., sim_backend=...)` before `setup()`, or use env vars below.

---

## Functions

| Function | Description |
|----------|-------------|
| `U.setup()` | Connect or sim; move to joint home; return `robot`. **Simulation:** interpolated homing. |
| `U.teardown(robot)` | Joint home; close port or sim. **Simulation:** interpolated home when applicable. |
| `U.move_joints(robot, q)` | Body-frame `[q1..q4]` (degrees). **Simulation:** smooth joint interpolation; hardware uses one setpoint. |
| `U.moveMagician(robot, q)` | Alias -> `move_joints`. |
| `U.moveMG400(robot, q)` | Alias -> `move_joints`. |
| `U.move_and_get_feedback(robot, q)` | Move then `(x, y, z, r)`. |
| `U.get_joints(robot)` | Body-frame joints (degrees). |
| `U.get_pose(robot)` | `(x, y, z, r)` mm / deg. |
| `U.safe_move(robot, x, y, z, r, mode="J")` | Clamped Cartesian move. |
| `U.body_to_firmware_angles(q, robot_type=None)` | Firmware quadruple (MG400 J3 coupling). |
| `U.firmware_to_body_angles(...)` | Inverse of the above. |
| `U.configure(...)` | Optional pre-`setup()` overrides. |
| `U.set_tool_offset(bot_inner, ox, oy, oz)` | Magician EEPROM TCP. Real Magician: `robot.handles["bot"]`. |
| `U.get_tool_offset(bot_inner)` | Read TCP offset (Magician). |

---

## Constants (after `configure` / `setup`)

| | Magician | MG400 |
|---|----------|--------|
| `L1`, `L2` | 135, 147 mm | 175, 175 mm |
| `Z_base` (in `ROBOT_MODELS`) | 103.0 mm | 116.0 mm |

**Magician FK in `myCode.fk_predict`:** uses **no added `Z_base`** in *z* (shoulder-frame formula matching firmware / tool `none` home **(147, 0, 135)** mm at body `(0,0,0,0)`).

**MG400 FK in `myCode.fk_predict`:** uses **`U.MG400_API_*`** constants so predicted **(x,y,z)** matches **simulator / `GetPose`** (see `simulation/runtime/kinematics.py` `fk_mg400_api`). The planar **`ROBOT_MODELS`** `L1`/`L2`/`Z_base` row is for teaching / Lab 2 Jacobian derivations, not this pose check.

---

## Joint convention

- **Magician:** body-frame matches firmware; J3 absolute world angle (0 = forearm horizontal).
- **MG400:** body `q`; firmware **`J3_fw = J2 + q3`**.

---

## Environment (this lab)

| Variable | Default if unset | Effect |
|----------|------------------|--------|
| `DOBOT_SIMULATION` | Simulation **on** | `0` / `false` / `no` / `off` for real hardware. |
| `DOBOT_ROBOT_TYPE` | `magician` | `mg400` for MG400. |
| `DOBOT_SIM_BACKEND` | `mujoco` | `pybullet` optional. |
| `DOBOT_EE` | `none` | Magician sim: `none` / `motor` / `suction`. |
| `DOBOT_VIZ` | GUI on | `0` = headless. |
| `DOBOT_SIM_HOLD` | off | `1` = keep the sim window open until Enter when possible. |
| `DOBOT_SIM_HOLD_SECS` | unset | Keep the sim window open for this many seconds. |
| `DOBOT_SIM_RUNTIME` | - | Override `simulation/runtime` path. |
| `DOBOT_MG400_ROBOT` | `1` | Station `1`–`4` (real MG400). |
| `DOBOT_MG400_IP` | from bundle | Override IP. |
| `DOBOT_PORT` | auto | Real Magician serial. |

```bash
python3 interface.py
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
DOBOT_SIM_BACKEND=pybullet DOBOT_VIZ=0 python3 interface.py
DOBOT_SIMULATION=0 python3 interface.py
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=1 python3 interface.py
```

---

## Kinematics (for `fk_predict`)

### Magician

```
reach  = L1·sin(q2) + L2·cos(q3)
z      = L1·cos(q2) − L2·sin(q3)
x      = reach·cos(q1)
y      = reach·sin(q1)
```

(`q` body-frame, degrees in API; use radians inside trig.)

### MG400 (compare to ``get_pose`` / simulator)

Lab 1 uses the **API-frame** model (same as ``fk_mg400_api`` in ``simulation/runtime/kinematics.py``), not the planar teaching ``L1``/``L2``/``Z_base`` row in ``ROBOT_MODELS``:

Constants: ``U.MG400_API_C_R``, ``U.MG400_API_L1``, ``U.MG400_API_L2``, ``U.MG400_API_C_Z``.

```text
j3_fw = q2 + q3
reach = C_R + L1·sin(q2) + L2·cos(j3_fw)
z     = C_Z + L1·cos(q2) − L2·sin(j3_fw)
x     = reach·cos(q1)
y     = reach·sin(q1)
```

For a **planar 2R teaching** derivation (Jacobian Lab 2 style), use ``U.L1``, ``U.L2``, ``U.Z_base`` from ``ROBOT_MODELS`` instead - that model is not identical to this API-frame pose.

---

## Example

```python
import utils as U

robot = U.setup()
try:
    U.move_joints(robot, [0, 30, 20, 0])
    x, y, z, r = U.get_pose(robot)
    print(f"X={x:.1f} Y={y:.1f} Z={z:.1f}")
finally:
    U.teardown(robot)
```
