# ME403 Student API Reference

Single reference for bundled exercise helpers. Keep this open while editing `myCode.py`.

**Canonical file:** `dobot-robotics-lab/docs/student_api.md`. **Shortcuts:** some exercises ship a shorter `student_api.md` beside `myCode.py` (trimmed to that tree only).

**Imports:** Run Python **inside** the exercise directory (for example under `labs/`) so `import utils as U` resolves. If you run from elsewhere, set `PYTHONPATH` to that directory or `cd` there first.

---

## Which `utils.py`?

| Location | Robots | Session variable | Notes |
|----------|--------|------------------|--------|
| `labs/lab01_forward_kinematics/utils.py` | Magician or MG400 | `robot` | Same unified API as Lab 2; Lab 1 MG400 uses joint home `(0,0,0,0)` in `setup()`/`teardown()`. |
| `labs/lab02_jacobian_inverse_kinematics/utils.py` | Magician or MG400 | `robot` | Unified API; `move_joints`, `safe_move`, `configure`. |

Always `import utils as U` from the same directory as that exercise’s `interface.py` / `myCode.py`.

---

## Lifecycle

```python
import utils as U

session = U.setup()
try:
    # ... your code ...
    pass
finally:
    U.teardown(session)   # pass the handle returned by setup() (`robot` in unified Lab 1 / Lab 2).
```

`setup()` connects (or starts simulation), clears alarms where applicable, and moves to a safe pose (joint home or ready pose depends on the exercise — see that folder’s `student_api.md`). `teardown()` should run in a `finally` block so the serial port or simulator is released.

---

## Functions

Names refer to `U.*` after `import utils as U`.

| Function | Argument(s) | Returns | Description |
|----------|-------------|---------|-------------|
| `setup` | optional kwargs (`configure` in unified trees) | `bot` or `robot` | Start session (see table above). |
| `teardown` | `bot` or `robot` | — | Safe shutdown; always call in `finally`. |
| `moveMagician` | `bot`, `q=[q1..q4]` | — | **Magician-only utils:** joint move (degrees, firmware frame). |
| `move_joints` | `robot`, `q` | — | **Unified utils:** joint move (body-frame degrees, both robots). |
| `move_and_get_feedback` | `bot` or `robot`, `q` | `(x,y,z,r)` | Move then read pose (mm / deg). |
| `get_joints` | `bot` or `robot` | `(q1,q2,q3,q4)` deg | Current joints (semantics depend on module; see below). |
| `get_pose` | `bot` or `robot` | `(x,y,z,r)` | Cartesian TCP pose. |
| `safe_move` | `robot`, `x,y,z,r`, `mode=` | — | **Unified utils:** clamped Cartesian move; `mode="J"` or `"L"`. |
| `body_to_firmware_angles` | `q`, `robot_type?` | `(j1..j4)` | **Unified utils:** body-frame → firmware (mainly MG400 J3). |
| `configure` | keyword args | — | **Unified utils only:** `robot_type`, `use_simulation`, `sim_backend`, … before `setup()`. |
| `set_tool_offset` | Magician handle `bot_inner`†, `ox,oy,oz` | — | EEPROM TCP offset (Magician only). |
| `get_tool_offset` | Magician `bot_inner`† | `(ox,oy,oz)` | Read TCP offset (Magician only). |

† **Magician-only session:** `bot_inner = bot`. **Unified session, real Magician:** `bot_inner = robot.handles["bot"]`.

**Unified utils aliases:** `moveMagician(robot, q)` and `moveMG400(robot, q)` both call `move_joints`.

---

## Constants (after `setup()`)

`U.L1`, `U.L2`, `U.Z_base`, `U.READY_POSE`, `U.JOINT_BOUNDS_FW`, `U.SAFE_BOUNDS` come from the **`utils.py` you imported**.

**Magician geometry:** upper arm `L1 = 135` mm, forearm `L2 = 147` mm. Home TCP at joint zero with tool `none`: **(147, 0, 135)** mm.

**`Z_base` and lengths** come from **`ROBOT_MODELS`** in each unified `utils.py` (`labs/lab01_forward_kinematics`, `labs/lab02_jacobian_inverse_kinematics`): Magician **Z_base = 103** mm (metadata / MG400-style consistency), **MG400** **Z_base = 116** mm, **L1 = L2 = 175** mm. In **Lab 1** `myCode.fk_predict`, the **Magician** branch does **not** add `Z_base` to *z* (shoulder firmware frame; home **(147, 0, 135)** mm at body `(0,0,0,0)` with tool `none`). The **MG400** branch **does** use `U.Z_base` in *z*. For Jacobians (Lab 2), follow **`lab2_guide.pdf`** and take numeric values only from **that** exercise’s `utils.py`.

**Magician TCP presets** (same dict in both trees):

| Key | Offset (mm) | Home (0,0,0,0) TCP |
|-----|-------------|----------------------|
| `MAGICIAN_TOOL_OFFSETS["none"]` | (0, 0, 0) | (147, 0, 135) |
| `["motor"]` | (60, 0, 0) | (207, 0, 135) |
| `["suction"]` | (60, 0, −70) | (207, 0, 65) |

**Ready poses (Cartesian, unified utils):** Magician `(200, 0, 100, 0)`; MG400 `(300, 0, 50, 0)` — see `U.READY_POSE` after `setup()`.

---

## Joint convention (short)

- **Magician:** firmware angles match `GetPose`; J3 is absolute world angle (0 = forearm horizontal).
- **MG400:** command **body-frame** `q`; firmware **`J3_fw = J2 + J3_body`**. Use `body_to_firmware_angles` when you need explicit firmware quadruples. (Unified Lab 1 and Lab 2 `utils.py`.)

---

## Environment variables

| Variable | Values | Default (when unset) | Effect |
|----------|--------|----------------------|--------|
| `DOBOT_SIMULATION` | truthy / `0`,`false`,`no`,`off` | **Simulation on** | Real hardware only if explicitly disabled. |
| `DOBOT_ROBOT_TYPE` | `magician`, `mg400` | `magician` | Robot family (**unified** exercise stack). |
| `DOBOT_SIM_BACKEND` | `mujoco`, `pybullet` | `mujoco` | Simulation engine. |
| `DOBOT_EE` | `none`, `motor`, `suction` | `none` | Magician TCP in simulation. |
| `DOBOT_VIZ` | `1`, `0` | `1` | GUI vs headless (sim). |
| `DOBOT_SIM_RUNTIME` | path | — | Force `simulation/runtime` search path. |
| `DOBOT_SIM_CACHE` | path | `~/.cache/dobot_sim` | URDF prep cache. |
| `DOBOT_PORT` | e.g. `/dev/ttyUSB0` | — | Magician serial override. |
| `DOBOT_MG400_IP` / `MG400_IP` | IP string | packaged defaults | Real MG400 IP override (**unified** stack). |
| `DOBOT_MG400_ROBOT` | `1`–`4` | `1` | MG400 station index (**real** robot). |

### Shell quick reference

```bash
# From an exercise folder — Magician simulation (default)
python3 interface.py

# MG400 simulation (Lab 1 or Lab 2 unified `utils.py`)
DOBOT_ROBOT_TYPE=mg400 python3 interface.py

# PyBullet, headless, real robots
DOBOT_SIM_BACKEND=pybullet python3 interface.py
DOBOT_VIZ=0 python3 interface.py
DOBOT_SIMULATION=0 python3 interface.py
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=2 python3 interface.py
```

---

## Kinematics (formulas)

### Dobot Magician

Angles in radians in the algebra; joint commands use degrees in the API.

```
reach  = L1·sin(q2) + L2·cos(q3)
z      = L1·cos(q2) − L2·sin(q3)
x      = reach·cos(q1)
y      = reach·sin(q1)
```

### DOBOT MG400 (unified utils)

```
R   = L1·cos(q2) + L2·cos(q3_fw − q2)
z   = Z_base + L1·sin(q2) + L2·sin(q3_fw − q2)
x   = R·cos(q1)
y   = R·sin(q1)
```

with `q3_fw = q2 + q3` (body/firmware coupling). Use `U.Z_base`, `U.L1`, `U.L2` from **your** unified exercise `utils.py` (Lab 1 or Lab 2). **MG400 safe Z:** 5–140 mm (0 = table).

**Lab 1 note:** `myCode.fk_predict` compares to **`GetPose` / simulator** using **`U.MG400_API_*`** (API-frame model from `simulation/runtime/kinematics.py`), not this planar teaching row. Lab 2 Jacobian work may still use the planar `ROBOT_MODELS` lengths — follow your PDF.

---

## Examples

**Unified (`robot`) — Lab 1 FK or Lab 2 (Magician or MG400)**

```python
import utils as U

robot = U.setup()
try:
    U.move_joints(robot, [0, 30, 20, 0])
    x, y, z, r = U.get_pose(robot)
    print(f"X={x:.1f}  Y={y:.1f}  Z={z:.1f}")
finally:
    U.teardown(robot)
```

---

## See also

[`setup.md`](setup.md) · [`simulation.md`](simulation.md) · [`magician_setup.md`](magician_setup.md) · [`mg400_setup.md`](mg400_setup.md) · [`troubleshooting.md`](troubleshooting.md)
