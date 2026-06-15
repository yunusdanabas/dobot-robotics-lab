# Getting Started

Prepared for **ME403 Introduction to Robotics** by **Yunus Emre Danabas**.

This page is a short quickstart. The canonical documentation entry point is
`README.md` at the package root.

Prerequisite: `git` and internet access are required to fetch simulation assets.

## 1. Install Python Packages

```bash
cd dobot-robotics-lab
python3 -m pip install -r requirements/base.txt
```

## 2. Set Up Simulation

```bash
python3 scripts/bootstrap.py --simulation
```

This installs simulation packages and downloads upstream robot assets into
`vendor/`. URDF backends then prepare/cache runtime-ready copies on first use.

## 3. Run A Lab

```bash
cd labs/lab01_forward_kinematics
python3 interface.py
```

The default robot is the Magician in MuJoCo simulation. To use MG400 simulation:

```bash
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
```

To use PyBullet instead of MuJoCo:

```bash
DOBOT_SIM_BACKEND=pybullet python3 interface.py
```

## 4. Optional: Use Real Hardware

Only do this after simulation works.

Magician:

```bash
python3 ../../scripts/check_magician.py
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=magician python3 interface.py
```

MG400:

```bash
python3 ../../scripts/check_mg400.py --robot 1
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=1 python3 interface.py
```

## 5. If Something Is Missing

From `dobot-robotics-lab/`, run:

```bash
python3 scripts/bootstrap.py --full
```

For environment-variable details and troubleshooting, see:

- `docs/student_api.md` (full API for both labs; each `labs/*/` folder has a shorter lab-only `student_api.md`)
- `docs/simulation.md`
- `docs/troubleshooting.md`
