# Labs

Each lab folder contains the files students need for that lab.

Lab folders include a **lab-specific** `student_api.md` (quick reference for that `utils.py`). The merged reference is [`../docs/student_api.md`](../docs/student_api.md) (paths relative to this `labs/` folder).

General rule: edit `myCode.py` unless your instructor says otherwise.

```bash
cd labs/<exercise_folder>
python3 interface.py
```

Use the folder your instructor assigns (for example under `labs/` in this bundle).

| Lab folder | Topic |
|------------|-------|
| `lab01_forward_kinematics` | Forward kinematics |
| `lab02_jacobian_inverse_kinematics` | Jacobian and inverse kinematics |
| `lab03_path_planning` | Path planning (APF, PRM/RRT) — handout only for now |
| `lab04_visual_servoing` | Camera calibration and visual servoing |

Lab 1 (forward kinematics) and Lab 2 (Jacobian / IK) both use a **unified stack** (**Magician or MG400**). Each folder’s `README.md` and `student_api.md` describe that exercise’s `utils.py` and environment variables.

Lab 4 uses Magician hardware plus a USB webcam; see `lab04_visual_servoing/README.md` for the calibration workflow.
