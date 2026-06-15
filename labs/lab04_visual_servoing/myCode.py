import cv2
import numpy as np
import scipy as sci
import utils as U

# ============================================================
# USER SETTINGS
# ============================================================

CAMERA_INDEX = 0
CALIBRATION_FILE = "camera_calibration.npz"
MIN_CONTOUR_AREA = 500

# ============================================================
# HELPER FUNCTION
# ============================================================

def find_red_object(frame_bgr):
    """
    Detect the largest red object in the frame.
    Returns:
        center: (cx, cy) or None
        contour: contour or None
        mask: binary mask
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    # Red wraps around the hue axis in HSV, so we use two ranges.
    lower_red1 = np.array([0, 100, 80])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Morphological cleanup
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]

    if len(contours) == 0:
        return None, None, mask

    largest = max(contours, key=cv2.contourArea)

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, largest, mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return (cx, cy), largest, mask

# ============================================================
# MAIN LOOP
# ============================================================

def main():
    # Keep calibration loading inside main() so other scripts can import
    # find_red_object without requiring camera_calibration.npz.
    data = np.load(CALIBRATION_FILE)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    print("Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame.")
                break

            h, w = frame.shape[:2]

            # Undistort using calibration
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

            center, contour, mask = find_red_object(undistorted)

            display = undistorted.copy()

            # Desired image center
            u0 = display.shape[1] // 2
            v0 = display.shape[0] // 2
            cv2.circle(display, (u0, v0), 6, (255, 0, 0), -1)
            cv2.putText(display, "Image Center", (u0 + 10, v0 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            if contour is not None:
                cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)

            if center is not None:
                cx, cy = center
                cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(display, f"Cube centroid: ({cx}, {cy})", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                ex = cx - u0
                ey = cy - v0
                cv2.putText(display, f"Error: ex={ex}, ey={ey}", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                cv2.line(display, (u0, v0), (cx, cy), (255, 255, 0), 2)

            cv2.imshow("Red Cube Detection", display)
            cv2.imshow("Red Mask", mask)

            # ============================================================
            # Visual Servoing Code Should be Here!




            # ============================================================

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
