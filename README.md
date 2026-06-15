# ME403 Lab Files

Prepared for **ME403 Introduction to Robotics** by **Yunus Emre Danabas**.

This package is the student-facing workspace for Dobot Magician and MG400 labs.
Use this file as the canonical quickstart.

## Quickstart (Simulation First)

Use `python3` on Linux/macOS if `python` is not available. On Windows, use
`py -3` or the Python executable from your virtual environment.
You need `git` installed with internet access for asset fetch during bootstrap.

```bash
cd dobot-robotics-lab
python3 -m pip install -r requirements/base.txt
python3 scripts/bootstrap.py --simulation
cd labs/lab01_forward_kinematics
python3 interface.py
```

Simulation is the default path. Use real robots only after your code behaves
correctly in simulation.

Before any real Magician run, calibrate/home with DobotStudio (DOBOT Magician)
v1.9.4, then disconnect it before starting Python.
Only one controller process can own the robot at a time.

Set the TCP offset to `(0, 0, 0)` for FK labs. Two ways:
- **DobotStudio**: set end-effector type to `None`, advanced XYZ bias to `0,0,0`.
- **Python** (after initial DobotStudio homing):
  `U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["none"])`

See `docs/magician_setup.md` for the full end-effector mode reference.

## What `bootstrap --simulation` Does

- Installs simulation dependencies (`MuJoCo` and `PyBullet` profiles).
- Downloads upstream robot description assets into `vendor/`.
- Prepares you to run labs and simulation demos from this package root.

Note: URDF backends use runtime-prepared/cached copies of the vendored URDFs.
See `docs/simulation.md` for details on preprocessing and cache behavior.

## Simulation Environment Quick Reference

- `DOBOT_SIMULATION=1` enables simulation mode (default for labs).
- `DOBOT_SIM_BACKEND=mujoco|pybullet` selects backend (`mujoco` default).
- `DOBOT_VIZ=0` disables GUI windows (headless runs).
- `DOBOT_ROBOT_TYPE=magician|mg400` selects robot family in lab interfaces.
- `DOBOT_SIM_CACHE=/custom/path` overrides default cache (`~/.cache/dobot_sim`).
- `DOBOT_SIM_RUNTIME=/path/to/simulation/runtime` overrides search for bundled simulation code (Lab 1 / Lab 2).
- `DOBOT_MAGICIAN_URDF_PATH=/path/to/magician_none.urdf` overrides Magician
  URDF directly; folder overrides should contain the three mode variants.
- `DOBOT_MG400_URDF_PATH=/path/to/mg400.urdf` overrides MG400 URDF source
  path (useful for custom/modified URDF workflows).

## Common Commands

```bash
# Optional: install everything (simulation + hardware profiles)
python3 scripts/bootstrap.py --full

# Re-fetch upstream assets into vendor/
python3 scripts/fetch_assets.py --all

# MG400 network check
python3 scripts/check_mg400.py --robot 1

# Magician USB check
python3 scripts/check_magician.py
```

## Folder Map

| Folder | What it contains |
|--------|------------------|
| `labs/` | Lab starter files (`myCode.py`, `interface.py`, `utils.py`). |
| `simulation/` | Runtime, demos, and tests for simulation workflows. |
| `robots/` | Optional hardware-focused scripts/helpers. |
| `tools/` | Optional debugging and Windows support utilities. |
| `requirements/` | Install profiles for simulation and hardware tracks. |
| `scripts/` | Bootstrap/install and asset fetch helpers. |
| `vendor/` | Upstream third-party SDK and description checkouts. |
| `docs/` | Setup, simulation notes, and troubleshooting docs. |

## Additional Docs

- `docs/setup.md` for a short setup checklist.
- `docs/student_api.md` for the full API across labs. Each `labs/*/` folder also has its own shorter `student_api.md`.
- `docs/simulation.md` for backend behavior, URDF preprocessing, and cache flow.
- `docs/magician_setup.md` and `docs/mg400_setup.md` for hardware workflows.
- `docs/troubleshooting.md` for common setup/runtime failures.
- `docs/student_simulation_pack_guide.pdf` for the release explainer handout.
- `docs/magician_calibration_and_setup_guide.pdf` for Magician calibration and
  software-based homing workflow.

## Student Release Notes

- Third-party payloads in `vendor/` are included in this package.
- First simulation runs may still take longer because assets are preprocessed
  into cache.
- If you publish a minimal checkout without vendor payload, run:
  `python3 scripts/bootstrap.py --simulation` from package root.

## Acknowledgements

This teaching package builds on Dobot robot platforms, Dobot SDK resources,
upstream robot description assets, and the Python robotics/simulation ecosystem
including MuJoCo, PyBullet, NumPy, SciPy, Matplotlib, PyQt, and pyqtgraph.
Thanks to the maintainers of these tools and resources.
