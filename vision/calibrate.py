import cv2
import numpy as np
import glob

# 1. Define checkerboard dimensions
checkerboard_size = (8, 5)  # (columns, rows of internal corners)
square_size = 1.0  # any unit (e.g. 1.0 cm)

# 2. Prepare object points like (0,0,0), (1,0,0), ..., (8,5,0)
objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
objp[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2)
objp *= square_size

# 3. Arrays to store points
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

# 4. Load calibration images
images = glob.glob('calibration_imgs/*.jpg')  # Put your checkerboard images here
print(f"Found {len(images)} images.")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# 5. Calibrate camera
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera intrinsic matrix K:\n", K)
