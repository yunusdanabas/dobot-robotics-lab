# Lab 04 - API reference (`utils.py`)

Visual servoing lab — **Dobot Magician only** (USB serial). Run scripts from this folder.

---

## Session

```python
import utils as U

bot = U.setup()
try:
    x, y, z, r = U.move_and_get_feedback(bot, [q1, q2, q3, q4])
finally:
    U.teardown(bot)
```

Optional port override: set `DOBOT_PORT` or pass `port=` to `setup()`.

---

## Functions

| Function | Description |
|----------|-------------|
| `U.setup(port=None)` | Connect, clear alarms, move to joint home; return `bot`. |
| `U.teardown(bot)` | Move home and close the serial connection. |
| `U.move_and_get_feedback(bot, q)` | Body-frame joint angles `[q1..q4]` (deg); move and return `(x, y, z, r)`. |

Joint values are converted to firmware angles internally. If any converted joint is outside the safe limits, the helper denies that motion instead of sending it.

---

## Related scripts

| Script | Purpose |
|--------|---------|
| `get_aruco_board.py` | Write `charuco_board.png` for printing. |
| `capture_charuco_images.py` | Save frames under `charuco_images/`. |
| `calibrate_charuco.py` | Write `camera_calibration.npz`. |
| `myCode.py` | Main visual servoing loop (uses calibration file). |
