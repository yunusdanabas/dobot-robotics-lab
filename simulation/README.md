# Simulation

This folder contains the full simulation files used by the labs.

Backends:

- `mujoco`: default, full URDF-based simulation with MuJoCo.
- `pybullet`: optional URDF-based simulation with PyBullet.

Magician end-effector modes:

- `none`: bare flange, TCP offset `(0, 0, 0)` mm
- `motor`: legacy motor flange, TCP offset `(+60, 0, 0)` mm
- `suction`: suction cup mode, TCP offset `(+60, 0, -70)` mm (physical cup tip, 70 mm below motor shaft)

Set with `DOBOT_EE=none|motor|suction` or `--ee-mode` in Magician teleops.

Install and download assets from the package root:

```bash
python3 scripts/bootstrap.py --simulation
```

Runtime note:

- `vendor/` holds upstream robot description assets.
- URDF backends use prepared cached copies produced by
  `simulation/runtime/urdf_loader.py`.

Run a lab in simulation:

```bash
cd labs/<exercise_folder>
python3 interface.py
```

Headless mode:

```bash
DOBOT_VIZ=0 python3 interface.py
```

Heavy URDF assets are downloaded into `vendor/` by `scripts/fetch_assets.py`.
For details and troubleshooting, see `../docs/simulation.md` and
`../docs/troubleshooting.md`.
