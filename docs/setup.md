# Setup

Use this short checklist from the package root.

**Imports:** Snippets that use `import utils as U` assume you run Python **from the exercise folder** under `labs/` (or put that folder on `PYTHONPATH`). See [`student_api.md`](student_api.md) and any `student_api.md` shipped next to the exercise.

Prerequisite: `git` and internet access are required for asset fetch.

```bash
cd dobot-robotics-lab
python3 -m pip install -r requirements/base.txt
python3 scripts/bootstrap.py --simulation
```

Then open the exercise you need:

```bash
cd labs/<exercise_folder>
python3 interface.py
```

If you only received a PDF, download or clone `dobot-robotics-lab` first from the
course page or the link provided by your instructor.

If you use a real Dobot Magician, calibrate/home first with DobotStudio
(DOBOT Magician) v1.9.4, then disconnect it before running Python scripts.

For the Magician workflow in this material, use the arm without an extra end effector unless instructed. The TCP offset should be `(0, 0, 0)` for bare-flange forward kinematics. Two ways to do this:

**Option A — DobotStudio (one-time):**
- Select end-effector type `None` in DobotStudio.
- In advanced end-effector settings, set XYZ bias to `0, 0, 0`.

**Option B — Python (after initial DobotStudio homing):**
```python
import utils as U
bot = U.setup()
U.set_tool_offset(bot, *U.MAGICIAN_TOOL_OFFSETS["none"])   # sets (0, 0, 0) mm
ox, oy, oz = U.get_tool_offset(bot)                        # verify
print(f"TCP offset: ({ox:.1f}, {oy:.1f}, {oz:.1f}) mm")
```
See [`magician_setup.md`](magician_setup.md) for the full preset table and explanation.

For backend, cache, URDF runtime, and **all** simulator env vars in one place, read [`simulation.md`](simulation.md).  
For failures (missing meshes, GUI, network, serial), read [`troubleshooting.md`](troubleshooting.md).
