# DOBOT MG400 Helpers

The MG400 uses Ethernet/TCP.

Useful files:

| File | Purpose |
|------|---------|
| `00_connectivity_check.py` | Check ping, TCP ports, dashboard status, and errors. |
| `00_connectivity_gui.py` | GUI for connectivity checks and safe demo moves. |
| `00_raw_dashboard_probe.py` | Low-level dashboard protocol probe. |
| `01_connect_test.py` | Basic connection smoke test. |
| `07_keyboard_teleop.py` | GUI-first teleoperation (keyboard fallback via `--mode keyboard`). |

Network reminder:

- Connect the Ethernet cable to the controller `LAN2` port.
- PC static IP: `192.168.2.100/24`
- Robot 1: `192.168.2.7`
- Robot 2: `192.168.2.10`
- Robot 3: `192.168.2.9`
- Robot 4: `192.168.2.6`

For real MG400 use, run:

From the `dobot-robotics-lab/` package root:

```bash
python3 scripts/fetch_assets.py --mg400-sdk
```
