import cv2
import numpy as np
import os
import glob

# ============================================================
# USER SETTINGS
# ============================================================

_HERE = os.path.dirname(os.path.abspath(__file__))

# Folder that contains captured calibration images (next to this script)
IMAGE_FOLDER = os.path.join(_HERE, "charuco_images")

# Output file
OUTPUT_FILE = os.path.join(_HERE, "camera_calibration.npz")

# ChArUco board settings
# These must match the board you print (see get_aruco_board.py).
SQUARES_X = 5          # number of chessboard squares in X direction
SQUARES_Y = 7          # number of chessboard squares in Y direction
SQUARE_LENGTH = 0.040  # meters
MARKER_LENGTH = 0.032  # meters

# Sparse detections can crash cv2.calibrateCamera on OpenCV 4.11+.
MIN_CHARUCO_CORNERS = 8

# ArUco dictionary
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# ============================================================
# CREATE BOARD
# ============================================================

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)
charuco_detector = cv2.aruco.CharucoDetector(board)

# ============================================================
# LOAD IMAGES
# ============================================================

image_paths = sorted(glob.glob(os.path.join(IMAGE_FOLDER, "*.png")) +
                     glob.glob(os.path.join(IMAGE_FOLDER, "*.jpg")) +
                     glob.glob(os.path.join(IMAGE_FOLDER, "*.jpeg")))

if len(image_paths) == 0:
    raise RuntimeError(f"No images found in folder: {IMAGE_FOLDER}")

all_object_points = []
all_image_points = []
image_size = None

print(f"Found {len(image_paths)} images.")

show_detections = os.environ.get("DISPLAY") is not None

for path in image_paths:
    img = cv2.imread(path)
    if img is None:
        print(f"Skipping unreadable image: {path}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if image_size is None:
        image_size = gray.shape[::-1]  # (width, height)

    charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(gray)

    if charuco_corners is not None and len(charuco_corners) >= MIN_CHARUCO_CORNERS:
        object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
        all_object_points.append(object_points.reshape(-1, 3).astype(np.float32))
        all_image_points.append(image_points.reshape(-1, 2).astype(np.float32))

        if show_detections:
            vis = img.copy()
            if marker_corners is not None and len(marker_corners) > 0:
                cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)
            cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids)
            cv2.imshow("Detected ChArUco Corners", vis)
            cv2.waitKey(150)
    else:
        n = 0 if charuco_corners is None else len(charuco_corners)
        print(f"Skipping {path} ({n} ChArUco corners; need >= {MIN_CHARUCO_CORNERS})")

if show_detections:
    cv2.destroyAllWindows()

if len(all_object_points) < 5:
    raise RuntimeError("Too few valid calibration images. Capture more images with better board visibility.")

# ============================================================
# CALIBRATE
# ============================================================

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    all_object_points,
    all_image_points,
    image_size,
    None,
    None,
)

print("\nCalibration completed.")
print(f"RMS reprojection error: {ret:.6f}")
print("Camera matrix:")
print(camera_matrix)
print("Distortion coefficients:")
print(dist_coeffs.ravel())

# ============================================================
# SAVE RESULTS
# ============================================================

np.savez(
    OUTPUT_FILE,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    rms=ret
)

print(f"\nSaved calibration to: {OUTPUT_FILE}")
