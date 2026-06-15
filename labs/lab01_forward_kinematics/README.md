# Lab 01: Forward Kinematics

Prepared for **ME403 Introduction to Robotics** by **Yunus Emre Danabas**.

Implement the tasks in `myCode.py`. The interface and helper functions are
provided so you can focus on the kinematics code.

## Files

| File | Purpose |
|------|---------|
| `myCode.py` | Your work goes here (`run(robot)`). |
| `interface.py` | Entry script: `setup()` -> `myCode.run(robot)` -> `teardown()` (no REPL). |
| `utils.py` | Magician or MG400: session, simulation, `move_joints`, FK constants. |
| `student_api.md` | This lab’s API cheat sheet; see also [`../../docs/student_api.md`](../../docs/student_api.md). |
| `lab1_guide.pdf` | Lab handout. |

## Run In Simulation

From this folder:

```bash
python3 interface.py
```

Simulation defaults to **MuJoCo**. Edit **`myCode.py`** for Tasks 0–3.

**MG400** simulation:

```bash
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
```

**PyBullet** backend (Magician or MG400):

```bash
DOBOT_SIM_BACKEND=pybullet python3 interface.py
```

Headless (when your sim stack supports it):

```bash
DOBOT_VIZ=0 DOBOT_SIM_GUI=0 python3 interface.py
```

Keep the simulation window open after the tasks finish:

```bash
DOBOT_SIM_HOLD=1 python3 interface.py
```

## Run On Real Robot

**Magician** (USB serial - close DobotStudio first):

```bash
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=magician python3 interface.py
```

**MG400**:

```bash
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=1 python3 interface.py
```

Use [`../../docs/mg400_setup.md`](../../docs/mg400_setup.md) for network and SDK layout.
