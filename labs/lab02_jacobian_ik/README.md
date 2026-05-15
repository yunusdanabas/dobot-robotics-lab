# Lab 02: Jacobian And Inverse Kinematics

Prepared for **ME403 Introduction to Robotics** by **Yunus Emre Danabas**.

Implement the tasks in `myCode.py`. The interface and helper functions are
provided so you can focus on Jacobian and IK code.

## Files

| File | Purpose |
|------|---------|
| `myCode.py` | Your implementation goes here. |
| `interface.py` | Tk GUI by default; `--mode terminal` for text REPL (move / run / quit). |
| `utils.py` | Robot and simulation helper functions. |
| `student_api.md` | This lab’s API cheat sheet; see also [`../../docs/student_api.md`](../../docs/student_api.md) for both labs. |
| `lab2_guide.pdf` | Lab handout. |

## Run In Simulation

From this folder:

```bash
python3 interface.py
```

The default is a small **Tk** window (joint fields, Move Once, Run Lab Code). For a text-only session: `python3 interface.py --mode terminal`.

MG400 simulation:

```bash
DOBOT_ROBOT_TYPE=mg400 python3 interface.py
```

PyBullet backend:

```bash
DOBOT_SIM_BACKEND=pybullet python3 interface.py
```

## Run On Real Robot

Magician:

```bash
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=magician python3 interface.py
```

MG400:

```bash
DOBOT_SIMULATION=0 DOBOT_ROBOT_TYPE=mg400 DOBOT_MG400_ROBOT=1 python3 interface.py
```

## Terminal mode (`--mode terminal`)

```text
move q1 q2 q3 q4    Move once and print pose
x (or execute)      Run myCode.run(robot)
q                   Quit safely
```

## Tasks

1. Build a 4x4 Jacobian.
2. Test small perturbations and report MSE.
3. Implement weighted Gauss-Newton IK and draw a square.
