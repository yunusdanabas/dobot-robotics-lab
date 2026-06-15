# Windows Quick Start

Use PowerShell or Windows Terminal from the `dobot-robotics-lab` package root.

## 1. Setup

Create and activate a virtual environment:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements\base.txt
```

If PowerShell blocks `Activate.ps1`, run once:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

Simulation setup:

```powershell
python scripts\bootstrap.py --simulation
```

Real-robot setup, only when needed:

```powershell
python scripts\bootstrap.py --magician
python scripts\bootstrap.py --mg400
```

## 2. Run A Lab

Default Magician simulation:

```powershell
Push-Location labs\lab01_forward_kinematics
python interface.py
Pop-Location
```

MG400 simulation:

```powershell
Push-Location labs\lab01_forward_kinematics
$env:DOBOT_ROBOT_TYPE = "mg400"
python interface.py
Remove-Item Env:\DOBOT_ROBOT_TYPE
Pop-Location
```

PyBullet instead of MuJoCo:

```powershell
Push-Location labs\lab01_forward_kinematics
$env:DOBOT_SIM_BACKEND = "pybullet"
python interface.py
Remove-Item Env:\DOBOT_SIM_BACKEND
Pop-Location
```

## 3. Magician Hardware

Checklist:

- Connect USB and power.
- Close DobotStudio before running Python.
- If the port changes, rerun the port check.

Check the port:

```powershell
python scripts\check_magician.py
```

Run helper scripts:

```powershell
python robots\magician\01_find_port.py
python robots\magician\02_first_connection.py
python robots\magician\03_safe_move_demo.py
python robots\magician\07_keyboard_teleop.py --no-viz
```

Force a specific COM port for the current shell:

```powershell
$env:DOBOT_PORT = "COM3"
python robots\magician\02_first_connection.py
```

## 4. MG400 Hardware

Lab robot IPs:

| Robot | IP |
|-------|----|
| 1 | `192.168.2.7` |
| 2 | `192.168.2.10` |
| 3 | `192.168.2.9` |
| 4 | `192.168.2.6` |

Set the PC Ethernet adapter to `192.168.2.100/24`.

Show adapters:

```powershell
.\tools\windows\Set-MG400StaticIp.ps1
```

Apply the standard address from an elevated PowerShell window:

```powershell
.\tools\windows\Set-MG400StaticIp.ps1 -InterfaceAlias "Ethernet" -Apply
```

Download the MG400 SDK:

```powershell
python scripts\fetch_assets.py --mg400-sdk
```

Check and teleoperate:

```powershell
python scripts\check_mg400.py --robot 1
python robots\mg400\01_connect_test.py --robot 1
python robots\mg400\07_keyboard_teleop.py --robot 1
```

Use a custom IP:

```powershell
python scripts\check_mg400.py --ip 192.168.2.77
python robots\mg400\01_connect_test.py --ip 192.168.2.77
```

## 5. Common Fixes

- `python` opens Microsoft Store: use `py -3`, activate `.venv`, or disable Python App Execution Aliases in Windows settings.
- `Activate.ps1` blocked: run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.
- `dobot_api.py not found`: run `python scripts\fetch_assets.py --mg400-sdk`.
- Magician port not found: close DobotStudio, replug USB, then run `python scripts\check_magician.py`.
- MG400 connection fails: confirm power, cable, PC static IP, and `ping <robot-ip>`.
- Teleop exits immediately: run it from an interactive terminal, not from an IDE output panel.
- Visualizer steals focus: keep the terminal focused, or run Magician teleop with `--no-viz` and MG400 teleop without `--viz`.
- `Set-MG400StaticIp.ps1 -Apply` fails: start Windows Terminal with "Run as Administrator" and confirm the interface alias with `Get-NetAdapter`.

## References

- [`../../README.md`](../../README.md)
- [`../../GETTING_STARTED.md`](../../GETTING_STARTED.md)
- [`../../docs/README.md`](../../docs/README.md)
- [`../../docs/mg400_setup.md`](../../docs/mg400_setup.md)
- [`../../docs/magician_setup.md`](../../docs/magician_setup.md)
