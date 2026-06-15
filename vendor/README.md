# Vendor Assets

This folder contains the third-party SDK and robot description sources used by
the student package.

After `python3 scripts/fetch_assets.py --all` (or `bootstrap.py` profiles that fetch assets), expect:

```text
vendor/magician_ros2_urdf/   Magician URDF variants and DAE meshes — simulation + parity with jkaniuka/magician_ros2
vendor/mg400_ros2/           Full ROS2 clone (HarvestX/MG400_ROS2); used as source when copying mg400_description
vendor/mg400_description/    MG400 URDF and meshes used by simulation (copied from mg400_ros2/mg400_description)
vendor/TCP-IP-4Axis-Python Dobot MG400 TCP/IP SDK (real robot, `--mg400-sdk` / `--full`)
```

Magician USB control uses **`pydobotplus`** from pip (`requirements/base.txt`
plus the hardware profile when needed); no extra Magician vendor clone is
required for the bundled lab scripts.

Refresh or rebuild vendor content from the `dobot-robotics-lab/` package root:

```bash
python3 scripts/fetch_assets.py --all
```

Flags (see `scripts/fetch_assets.py`): `--magician-urdf`, `--mg400-urdf`, `--mg400-sdk`, `--all`.

Notes:

- Magician fetch can be overridden:
  - `DOBOT_MAGICIAN_URDF_URL=<git-url>` (clone source)
  - `DOBOT_MAGICIAN_URDF_SOURCE_DIR=/path/to/magician_ros2_urdf` (copy from an existing tree that has `urdf/magician.urdf` and `meshes/dae/`; missing `magician_none.urdf`, `magician_motor.urdf`, and `magician_suction.urdf` files are generated automatically)

Do not rename the folders above; discovery code expects these paths.

Simulation backends prepare/cache runtime-ready URDF copies from these assets (mesh conversion and simulator compatibility). Prepared files go under `DOBOT_SIM_CACHE` (default: `~/.cache/dobot_sim`).
