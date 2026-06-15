# Lab 02 - API reference (`utils.py`)

Jacobian and IK lab - **Dobot Magician or MG400**. Use with `myCode.py`; `interface.py` loads your tasks from there.

Full two-lab reference: [`../../docs/student_api.md`](../../docs/student_api.md).

Run Python **from this folder** so `import utils as U` works (`cd labs/lab02_jacobian_inverse_kinematics`).

---

## Session

`U.setup()` returns **`robot`** (a `RobotSession`: type, simulation flag, backend, and `handles`). Pass **`robot`** into motion and pose calls. Always call `U.teardown(robot)` in `finally`.

```python
import utils as U

robot = U.setup()
try:
    ...
finally:
    U.teardown(robot)
```

Optional: `U.configure(robot_type=..., use_simulation=..., sim_backend=...)` **before** `setup()` (or use env vars below).

---

## Functions

| Function | Description |
|----------|-------------|
| `U.setup()` | Connect / sim; joint home (Magician) or **ready pose** (MG400); return `robot`. |
| `U.teardown(robot)` | Safe pose; close sockets or sim. |
| `U.move_joints(robot, q)` | Joint move: `[q1..q4]` **body-frame** degrees (both robots). |
| `U.moveMagician(robot, q)` | Alias -> `move_joints`. |
| `U.moveMG400(robot, q)` | Alias -> `move_joints`. |
| `U.move_and_get_feedback(robot, q)` | Move then `(x, y, z, r)`. |
| `U.get_joints(robot)` | Body-frame `(q1..q4)` degrees. |
| `U.get_pose(robot)` | Cartesian `(x, y, z, r)` mm / deg. |
| `U.safe_move(robot, x, y, z, r, mode="J")` | Clamped Cartesian move; `mode="L"` for linear. |
| `U.body_to_firmware_angles(q, robot_type=None)` | `(j1..j4)` firmware angles (MG400 J3 coupling). |
| `U.configure(...)` | Set `robot_type`, `use_simulation`, `sim_backend`, etc. before `setup()`. |
| `U.set_tool_offset(bot_inner, ox, oy, oz)` | Magician EEPROM TCP. **Real robot:** `bot_inner = robot.handles["bot"]`. |
| `U.get_tool_offset(bot_inner)` | Read offset; same **`robot.handles["bot"]`** on real Magician. |

Simulation Magician may still expose `robot.handles["bot"]` for symmetry; prefer high-level `move_joints` / `get_pose` on `robot`.

---

## Constants (after `setup()`)

Values come from the active model - use **`U.L1`, `U.L2`, `U.Z_base`, `U.READY_POSE`, `U.JOINT_BOUNDS_FW`, `U.SAFE_BOUNDS`** from this `utils.py`.

| | Magician | MG400 |
|---|----------|--------|
| `L1`, `L2` | 135, 147 mm | 175, 175 mm |
| `Z_base` | 103.0 mm | 116.0 mm |
| `READY_POSE` | (200, 0, 100, 0) | (300, 0, 50, 0) |

Home **(0,0,0,0)** body / firmware: Magician TCP **(147, 0, 135)** mm (tool `none`). MG400 nominal analytic home **≈ (350, 0, 116)** mm - Lab 01 uses the same joint home and documents expectations there.

`U.MAGICIAN_TOOL_OFFSETS` (`none` / `motor` / `suction`). MG400 uses bare TCP (no offset API).

**Lab 1** and **Lab 2** ship the same numeric **`ROBOT_MODELS`** in their respective `utils.py` files; always **`import utils as U` from your exercise folder** so lengths match your PDF.

**Do not** mix in constants from unrelated course snapshots - use only the bundled `utils.py` beside your `myCode.py`.

---

## Joint convention

- **Magician:** body-frame matches firmware; J3 absolute world angle (0 = forearm horizontal).
- **MG400:** body-frame `q`; firmware **`J3_fw = J2 + q3_body`**. Use `body_to_firmware_angles` when you need explicit firmware quadruples.

---

## Environment (this lab)

| Variable | Default if unset | Effect |
|----------|------------------|--------|
| `DOBOT_SIMULATION` | Simulation **on** | `0` / `false` / `no` / `off` for real hardware. |
| `DOBOT_ROBOT_TYPE` | `magician` | `mg400` for MG400. |
| `DOBOT_SIM_BACKEND` | `mujoco` | `pybullet` optional. |
| `DOBOT_EE` | `none` | Magician sim end-effector mode. |
| `DOBOT_VIZ` | GUI on | `0` = headless. |
| `DOBOT_SIM_RUNTIME` | - | Override `simulation/runtime` discovery. |
| `DOBOT_MG400_ROBOT` | `1` | Station `1`–`4` (real MG400). |
| `DOBOT_MG400_IP` | from bundle | Override robot IP if needed. |
| `DOBOT_PORT` | auto | Real Magician serial. |

```bash
python3 interface.py
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
DOBOT_SIM_BACKEND=pybullet DOBOT_VIZ=0 python3 interface.py
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=magician python3 interface.py
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=1 python3 interface.py
```

---

## Kinematics

### Magician

Same shoulder-frame FK as Lab 1 (radians in the algebra; API uses degrees):

```
reach  = L1·sin(q2) + L2·cos(q3)
z      = L1·cos(q2) − L2·sin(q3)
x      = reach·cos(q1)
y      = reach·sin(q1)
```

Use **`U.L1`, `U.L2`** from this folder. **`U.Z_base`** (103 mm for Magician in `ROBOT_MODELS`) is for Jacobian / unified metadata; add it to *z* only if your **`lab2_guide.pdf`** derivation does - for Magician planar FK without extra *z* offset, match the PDF. **Lab 1** Magician `fk_predict` omits `Z_base` in *z* by design.

### MG400 (API-frame model)

Lab 2 uses the same **API-frame** model as Lab 1 so predicted poses match `get_pose` and the simulator:

```
j3_fw = q2 + q3
reach = C_R + L1·sin(q2) + L2·cos(j3_fw)
z     = C_Z + L1·cos(q2) − L2·sin(j3_fw)
x     = reach·cos(q1)
y     = reach·sin(q1)
```

Constants: `U.MG400_API_C_R`, `U.MG400_API_L1`, `U.MG400_API_L2`, `U.MG400_API_C_Z`.

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

Real Magician TCP check:

```python
ox, oy, oz = U.get_tool_offset(robot.handles["bot"])
```
