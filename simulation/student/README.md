# Student Simulation Demos

These scripts are optional demos and teleops for simulation.

Examples:

From the `dobot-robotics-lab/` package root:

```bash
python3 simulation/student/magician_mujoco_teleop.py --help
python3 simulation/student/mg400_pybullet_teleop.py --help
```

Run demo commands from the package root so shared helpers (including
`terminal_keys.py`) resolve consistently.

Useful overrides:

- `DOBOT_SIM_BACKEND=pybullet|mujoco`
- `DOBOT_VIZ=0` for headless runs

Use lab `interface.py` first. These demos are for extra exploration.
