# Lab 04: Visual Servoing

Prepared for **ME403 Introduction to Robotics** by **Yunus Emre Danabas**.

Calibrate the end-effector webcam, detect a red target in the image, and implement visual servoing in `myCode.py`.

## Files

| File | Purpose |
|------|---------|
| `myCode.py` | Red-object detection loop; add visual servoing where marked. |
| `utils.py` | Magician connect / move / teardown helpers. |
| `get_aruco_board.py` | Generate printable ChArUco board (`charuco_board.png`). |
| `capture_charuco_images.py` | Save calibration frames to `charuco_images/`. |
| `calibrate_charuco.py` | Compute `camera_calibration.npz` from captured images. |
| `student_api.md` | Quick reference for `utils.py`. |
| `lab4_guide.pdf` | Lab handout (setup, calibration, tasks). |
| `requirements.txt` | Python dependencies for this lab. |

## Setup

From this folder:

```bash
python -m pip install -r requirements.txt
```

Close DobotStudio before connecting to the Magician over USB.

## Workflow

1. **Generate board:** `python get_aruco_board.py` — print `charuco_board.png` and measure square/marker sizes.
2. **Capture images:** `python capture_charuco_images.py` — save ~20 frames (`s` key) with the board at varied poses.
3. **Calibrate:** Match board dimensions in `calibrate_charuco.py`, then run it to produce `camera_calibration.npz`.
4. **Visual servo:** `python myCode.py` — implement the control loop in the marked section.

## Notes

- Board settings in `get_aruco_board.py` must match `calibrate_charuco.py`.
- Use measured printed dimensions, not the nominal values in the generator.
- Measure the camera–J4 offset **d** described in the handout before tuning the controller.
