# Simulation Tests

These tests help verify the simulation install.

From `ME403_LabFiles/`:

```bash
DOBOT_VIZ=0 python3 simulation/tests/test_07_backend_crossval.py
```

URDF-backend tests require assets downloaded by `scripts/fetch_assets.py`
(typically `test_04` and above). Basic FK/Cartesian tests can run without full
URDF payload depending on backend usage.
