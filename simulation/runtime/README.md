# Simulation Runtime

These are the full runtime modules used by `utils.py` in each lab.

They expect upstream robot description assets under:

```text
vendor/magician_ros2_urdf
vendor/mg400_description
```

Create those folders with:

From the `ME403_LabFiles/` package root:

```bash
python3 scripts/fetch_assets.py --all
```

URDF backends then call `urdf_loader.py` to generate prepared/cached runtime
copies (mesh conversion, URI rewriting, simulator compatibility updates). Cache
root defaults to `~/.cache/dobot_sim` and can be changed with `DOBOT_SIM_CACHE`.
