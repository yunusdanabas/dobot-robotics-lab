# Dobot Magician Helpers

The Magician uses USB serial.

Useful files:

| File | Purpose |
|------|---------|
| `01_find_port.py` | Find the USB serial port. |
| `02_first_connection.py` | Basic connection smoke test. |
| `03_safe_move_demo.py` | Safe motion demo. |
| `07_keyboard_teleop.py` | GUI-first teleoperation (keyboard fallback via `--mode keyboard`). |
| `00_magician_gui.py` | Simple GUI for joint control. |

Before real hardware:

- Connect USB and power.
- Calibrate/home first in DobotStudio (DOBOT Magician) v1.9.4.
- Disconnect/close DobotStudio before Python control.
- DobotStudio and Python scripts cannot control the same Magician at the same time.
- On Linux, run `sudo usermod -a -G dialout $USER`, then log out and back in.
