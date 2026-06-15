# Requirements

Use these files as small install profiles.

Run these commands from the `dobot-robotics-lab/` package root. Use `py -3` on
Windows if you are not inside an activated virtual environment.

```bash
python3 -m pip install -r requirements/base.txt
python3 -m pip install -r requirements/simulation_mujoco.txt
python3 -m pip install -r requirements/simulation_pybullet.txt
python3 -m pip install -r requirements/magician_hardware.txt
python3 -m pip install -r requirements/mg400_hardware.txt
```

Most students should start with:

```bash
python3 scripts/bootstrap.py --simulation
```
