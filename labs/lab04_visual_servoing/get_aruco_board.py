import cv2

SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH = 0.040   # meters
MARKER_LENGTH = 0.032   # meters
ARUCO_DICT = cv2.aruco.DICT_4X4_50

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)

# image size in pixels for printing
img = board.generateImage((1000, 1400))
cv2.imwrite("charuco_board.png", img)
print("Saved charuco_board.png")