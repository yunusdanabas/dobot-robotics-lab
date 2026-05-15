# Troubleshooting

## Simulation says URDF is missing

Run from `ME403_LabFiles/`:

```bash
python3 scripts/fetch_assets.py --all
```

If it still fails:

- Confirm `vendor/magician_ros2_urdf` and `vendor/mg400_description` exist.
- If imports say the simulation runtime is missing, set  
  `DOBOT_SIM_RUNTIME=/absolute/path/to/ME403_LabFiles/simulation/runtime`  
  or run from an unmodified `ME403_LabFiles` tree so exercise `utils.py` can find `simulation/runtime` automatically.
- Delete your simulation cache directory and retry:
  `DOBOT_SIM_CACHE` path if set, otherwise `~/.cache/dobot_sim`.

## Meshes are invisible or broken

- Some upstream Magician mesh files may be tiny pointer/stub files.
- Run `python3 scripts/fetch_assets.py --all` again to re-download meshes.
- Ensure simulation dependencies were installed:
  `python3 scripts/bootstrap.py --simulation`

## MG400 URDF generation fails

`fetch_assets.py` may require `xacro` to generate `mg400.urdf`.

```bash
python3 -m pip install xacro
python3 scripts/fetch_assets.py --mg400-urdf
```

## Magician serial port is missing

- Calibrate/home once in DobotStudio (DOBOT Magician) v1.9.4 before Python sessions.
- Close DobotStudio before running Python (single-controller rule).
- Check USB and power.
- On Linux, confirm your user is in `dialout`.
- Try `DOBOT_PORT=/dev/ttyUSB0`.

## MG400 does not connect

- Set PC static IP to `192.168.2.100/24`.
- Check Ethernet cable is plugged into controller `LAN2`.
- Ping the robot IP.
- Run `python3 scripts/check_mg400.py --robot 1` from `ME403_LabFiles/`.

## Real robot pose differs from simulation by ~60 mm in X

The Magician's TCP offset (end-effector mode) may differ between the real
robot and the simulation.

1. Check the current firmware offset from Python (run from your exercise directory; **first argument** to `get_tool_offset` must be the live `Dobot` — unwrap session wrappers so you pass the serial-connected instance):
   ```python
   import utils as U
   bot = U.setup()
   print(U.get_tool_offset(bot))   # bare-flange exercises: expect (0.0, 0.0, 0.0)
   ```
2. If it's not `(0, 0, 0)`, reset it:
   ```python
   U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["none"])
   ```
3. Ensure the simulation side also uses `DOBOT_EE=none` (the default).

A systematic 60 mm X offset means the real robot has `(+60, 0, 0)` set
(motor/suction mode) while simulation uses bare-flange mode, or vice versa.
See [`magician_setup.md`](magician_setup.md) for the full preset table.

## MuJoCo or GUI does not open

- Try headless mode first:

```bash
cd labs/<exercise_folder>
DOBOT_VIZ=0 python3 interface.py
```

- Install GUI dependencies:

```bash
python3 -m pip install PyQt5 pyqtgraph
```

- If MuJoCo display issues persist on Linux, try:

```bash
cd labs/<exercise_folder>
MUJOCO_GL=egl DOBOT_VIZ=0 python3 interface.py
```

---

## See also

[`simulation.md`](simulation.md) · [`setup.md`](setup.md) · [`student_api.md`](student_api.md) · [`magician_setup.md`](magician_setup.md) · [`mg400_setup.md`](mg400_setup.md)
