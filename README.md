# Dobot Robotics Lab

Prepared for **ME403 Introduction to Robotics** by **Yunus Emre Danabas**.

This package is the student workspace for Dobot Magician and MG400 robotics
exercises. It includes exercise starter files, robot helper APIs, simulation
support, setup scripts, and troubleshooting notes.

Use this README as the main entry point. Start in simulation, test your code,
then move to real hardware only when your simulation behavior is correct.

## Start Here

Prerequisites:

- Python 3.10 or newer.
- `git` with internet access for fetching robot assets.
- A terminal opened at the package root.

Linux/macOS examples use `python3`. On Windows, use `py -3` or the Python from
your virtual environment.

```bash
cd dobot-robotics-lab
python3 -m pip install -r requirements/base.txt
python3 scripts/bootstrap.py --simulation
```

Then open the exercise folder assigned by your instructor:

```bash
cd labs/<exercise_folder>
python3 interface.py
```

Most exercises use this pattern:

- Read the exercise `README.md`.
- Edit `myCode.py`.
- Run `python3 interface.py`.
- Keep the local `student_api.md` open while coding.

## Choose Robot Or Backend

The exercise interfaces default to Magician simulation with MuJoCo.

Use MG400 simulation:

```bash
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
```

Use PyBullet instead of MuJoCo:

```bash
DOBOT_SIM_BACKEND=pybullet python3 interface.py
```

Run without GUI windows:

```bash
DOBOT_VIZ=0 python3 interface.py
```

Useful environment variables:

| Variable | Purpose |
|----------|---------|
| `DOBOT_SIMULATION=1` | Use simulation mode. This is the default for exercises. |
| `DOBOT_ROBOT_TYPE=magician|mg400` | Select the robot family. |
| `DOBOT_SIM_BACKEND=mujoco|pybullet` | Select the simulator backend. |
| `DOBOT_VIZ=0` | Disable visualization windows for headless runs. |
| `DOBOT_SIM_CACHE=/path` | Override the simulation asset cache path. |
| `DOBOT_SIM_RUNTIME=/path` | Override where exercise helpers find the bundled simulation runtime. |

## Real Robot Use

Use real hardware only after simulation works.

Before a real Magician run:

- Calibrate/home with DobotStudio for Dobot Magician.
- Close DobotStudio before starting Python.
- Make sure only one controller process owns the robot.
- For forward-kinematics work, use TCP offset `(0, 0, 0)` unless instructed otherwise.

Set Magician TCP offset to `(0, 0, 0)` in one of these ways:

- DobotStudio: set end-effector type to `None`, then set advanced XYZ bias to `0,0,0`.
- Python: after initial DobotStudio homing, run `U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["none"])`.

Hardware checks from the package root:

```bash
python3 scripts/check_magician.py
python3 scripts/check_mg400.py --robot 1
```

Run an exercise on hardware from inside the exercise folder:

```bash
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=magician python3 interface.py
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=1 python3 interface.py
```

See `docs/magician_setup.md` and `docs/mg400_setup.md` before using real robots.

## Useful Commands

From the package root:

```bash
# Install all supported profiles, including simulation and hardware helpers.
python3 scripts/bootstrap.py --full

# Re-fetch upstream robot assets into vendor/.
python3 scripts/fetch_assets.py --all

# Check the local Python environment.
python3 scripts/check_install.py

# Check a Magician USB connection.
python3 scripts/check_magician.py

# Check an MG400 network connection.
python3 scripts/check_mg400.py --robot 1
```

## What Bootstrap Does

`python3 scripts/bootstrap.py --simulation` prepares the simulation workflow:

- Installs simulation dependencies for MuJoCo and PyBullet.
- Downloads upstream robot description assets into `vendor/`.
- Leaves runtime-prepared URDF files in a cache on first simulation use.

If simulation assets are missing or stale, rerun:

```bash
python3 scripts/fetch_assets.py --all
```

## Folder Map

| Folder | Contents |
|--------|----------|
| `labs/` | Exercise starter folders with `interface.py`, `myCode.py`, `utils.py`, and local API notes. |
| `simulation/` | Simulation runtime, student demos, and backend tests. |
| `robots/` | Optional hardware-focused examples and helper scripts. |
| `requirements/` | Install profiles for base, simulation, and hardware dependencies. |
| `scripts/` | Bootstrap, install checks, robot checks, and asset fetching. |
| `tools/` | Debugging utilities and Windows support helpers. |
| `vendor/` | Upstream SDK and robot description assets. |
| `docs/` | Setup, simulation, hardware, API, and troubleshooting guides. |

## More Documentation

Recommended reading order:

1. `GETTING_STARTED.md` for the shortest setup path.
2. `docs/setup.md` for the setup checklist.
3. `docs/student_api.md` for the shared helper API and environment variables.
4. `docs/simulation.md` for simulator backend and asset-cache details.
5. `docs/magician_setup.md` or `docs/mg400_setup.md` before hardware use.
6. `docs/troubleshooting.md` when setup or runtime behavior looks wrong.

Each exercise folder also has its own `README.md` and `student_api.md` with the
details for that exercise.

## Notes

- Simulation is the safest default and should be your first test path.
- Real robots require exclusive access. Close any other robot-control software first.
- First simulation startup may be slower because robot assets are prepared and cached.
- If a minimal checkout does not include `vendor/`, run `python3 scripts/bootstrap.py --simulation`.

## Acknowledgements

This teaching package builds on Dobot robot platforms, Dobot SDK resources,
upstream robot description assets, and the Python robotics/simulation ecosystem,
including MuJoCo, PyBullet, NumPy, SciPy, Matplotlib, PyQt, and pyqtgraph.
Thanks to the maintainers of these tools and resources.
