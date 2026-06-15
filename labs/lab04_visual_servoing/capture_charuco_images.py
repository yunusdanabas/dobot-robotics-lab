import cv2
import os

SAVE_FOLDER = "charuco_images"
CAMERA_INDEX = 0

os.makedirs(SAVE_FOLDER, exist_ok=True)

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam.")

print("Press 's' to save a frame.")
print("Press 'q' to quit.")

count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    display = frame.copy()
    cv2.putText(display, "Press 's' to save, 'q' to quit", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Capture ChArUco Images", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        filename = os.path.join(SAVE_FOLDER, f"img_{count:03d}.png")
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        count += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()