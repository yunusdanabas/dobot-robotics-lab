# Simulation Setup

Simulation is the recommended first step before using real hardware in teaching exercises.

From the `dobot-robotics-lab/` package root:

```bash
python3 scripts/bootstrap.py --simulation
```

## What This Downloads vs What Runs

- `scripts/bootstrap.py --simulation` fetches upstream description assets into
  `vendor/` (for example `vendor/magician_ros2_urdf` and
  `vendor/mg400_description`).
- URDF backends (`mujoco`, `pybullet`) load prepared cached copies generated
  from files under `vendor/`.
- `simulation/runtime/urdf_loader.py` prepares those runtime copies (mesh
  conversion, URI rewriting, simulator compatibility adjustments) and stores
  them in cache.
- Default cache path: `~/.cache/dobot_sim` (override with `DOBOT_SIM_CACHE`).

## Backend Selection

Default backend:

```text
MuJoCo
```

PyBullet option:

```bash
DOBOT_SIM_BACKEND=pybullet python3 interface.py
```

If you are unsure which backend to use:

- Start with `mujoco` (default) for course material.
- Use `pybullet` if MuJoCo graphics/drivers are unavailable.
- Use `DOBOT_VIZ=0` for headless runs, remote sessions, or CI-style checks.

Headless mode:

```bash
DOBOT_VIZ=0 python3 interface.py
```

## Common First-Run Delays

The first URDF backend run can be slower because it may:

- convert meshes (for simulator compatibility),
- build cached prepared URDF bundles.

## Asset Recovery

If a URDF or mesh file is missing, run from package root:

```bash
python3 scripts/fetch_assets.py --all
```

Runtime does not auto-download missing assets. If `vendor/` is incomplete, run
bootstrap/fetch explicitly before retrying exercise scripts.

## Useful Environment Variables

- `DOBOT_SIMULATION` — **simulation is the default when this variable is unset.** Set to `0`, `false`, `no`, or `off` only when using a real robot.
- `DOBOT_SIM_BACKEND=mujoco|pybullet` selects simulator backend.
- `DOBOT_EE=none|motor|suction` selects Magician EE/TCP mode in simulation.
  - `none`: `(0, 0, 0)` mm offset — home TCP `(147, 0, 135)` mm
  - `motor`: `(+60, 0, 0)` mm offset — home TCP `(207, 0, 135)` mm
  - `suction`: `(+60, 0, -70)` mm offset — home TCP `(207, 0, 65)` mm (physical cup tip)
    - Note: DobotStudio "suction cup" factory preset = `(+60, 0, 0)` (motor-shaft TCP). Use `DOBOT_EE=motor` to match DobotStudio `GetPose` readings.
- `DOBOT_VIZ=0` disables GUI windows.
- `DOBOT_ROBOT_TYPE=magician|mg400` selects the robot family in **stacks that support both** (Magician-only bundles ignore this).
- `DOBOT_MG400_ROBOT=1|2|3|4` selects the station IP map for **real** MG400 when using the packaged MG400 networking helpers; see [`mg400_setup.md`](mg400_setup.md).
- `DOBOT_MG400_IP` / `MG400_IP` overrides the default robot IP if your network differs.
- `DOBOT_SIM_CACHE=/custom/path` changes cache location.
- `DOBOT_SIM_RUNTIME=/path/to/simulation/runtime` overrides discovery of `simulation/runtime` when exercise `utils.py` imports the shared runtime.
- `DOBOT_MAGICIAN_URDF_PATH=/path/to/magician_none.urdf` overrides Magician URDF directly; pointing it to a description folder expects the three mode variants under `urdf/`.
- `DOBOT_MG400_URDF_PATH=/path/to/mg400.urdf` overrides MG400 URDF.
- Alias env vars are also supported:
  `DOBOT_MAGICIAN_URDF` and `DOBOT_MG400_URDF`.

For standalone teleops, use `--ee-mode {none,motor,suction}` on Magician
scripts under `simulation/student/` in the package root.

## Matching Sim and Real-Robot EE Mode

The simulation TCP mode (`DOBOT_EE`) and the real robot's TCP offset must
agree for FK/IK errors to reflect kinematics rather than an offset mismatch.

| Sim env var | Real robot state needed |
|-------------|------------------------|
| `DOBOT_EE=none` | `set_tool_offset(bot, 0, 0, 0)` or DobotStudio type=None |
| `DOBOT_EE=motor` | `set_tool_offset(bot, 60, 0, 0)` or DobotStudio type=Motor |
| `DOBOT_EE=suction` | `set_tool_offset(bot, 60, 0, -70)` — models physical cup tip (not DobotStudio preset) |

Python shortcut (`robots/magician/utils.py` and exercise **`utils.py`** modules):

```python
import utils as U
# First argument must be the live Dobot instance (unwrap from a session wrapper if needed).
U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["none"])  # match DOBOT_EE=none
```

For bare-flange teaching setups use `none` on both sides — positions match the
geometric model directly.

---

## See also

- [`setup.md`](setup.md) — first-time install from package root  
- [`student_api.md`](student_api.md) — exercise `utils.py` and env vars  
- [`magician_setup.md`](magician_setup.md) — TCP / DobotStudio alignment  
- [`troubleshooting.md`](troubleshooting.md) — missing URDF, meshes, GUI, `xacro`
