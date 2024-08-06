import cv2
import numpy as np
import glob
import pickle

# Get the list of calibration images
file_names = glob.glob("*.png")
pattern_size = (11 - 1, 8 - 1)
q = []

# Generate checkerboard (world) coordinates Q. The board has 25 x 18 fields with a size of 15x15mm
checkerboard = (11, 8)
objp = np.zeros(((checkerboard[1] - 1) * (checkerboard[0] - 1), 3), np.float32)
objp[:, :2] = np.mgrid[1:checkerboard[0], 1:checkerboard[1]].T.reshape(-1, 2)

Q = []

# Detect feature points
for i, fname in enumerate(file_names):
    print(f"Processing {fname}")
    
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    pattern_found, corners = cv2.findChessboardCorners(gray, pattern_size, 
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
    
    # Refine the found corner detections
    if pattern_found:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        q.append(corners)
        Q.append(objp)
        print("Pattern found and refined")
    else:
        print("Pattern not found")

    # Display
    cv2.drawChessboardCorners(img, pattern_size, corners, pattern_found)
    cv2.imshow('chessboard detection', img)
    cv2.waitKey(0)

# Check if we have enough points for calibration
if len(Q) == 0 or len(q) == 0:
    print("No patterns were detected in any image. Please check your images and pattern size.")
else:
    # Intrinsic camera matrix and distortion coefficients
    K = np.eye(3, dtype=np.float32)
    k = np.zeros(5, dtype=np.float32)

    # Calibration
    print("Calibrating...")
    flags = (cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_FIX_K3 +
             cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_PRINCIPAL_POINT)
    frame_size = (1440, 1080)

    ret, K, k, rvecs, tvecs = cv2.calibrateCamera(Q, q, frame_size, K, k, flags=flags)

    print(f"Reprojection error = {ret}\nK =\n{K}\nk =\n{k}")

    # Save calibration results
    calibration_data = {
        "cameraMatrix": K,
        "dist": k
    }

    with open('/home/naor/Desktop/naor/study/fhsbg/calibration/calibration.pkl', 'wb') as f:
        pickle.dump(calibration_data, f)

    with open('/home/naor/Desktop/naor/study/fhsbg/calibration/cameraMatrix.pkl', 'wb') as f:
        pickle.dump(K, f)

    with open('/home/naor/Desktop/naor/study/fhsbg/calibration/dist.pkl', 'wb') as f:
        pickle.dump(k, f)

    # Precompute lens correction interpolation
    mapX, mapY = cv2.initUndistortRectifyMap(K, k, None, K, frame_size, cv2.CV_32FC1)

    # Show lens corrected images
    for fname in file_names:
        print(fname)
        
        img = cv2.imread(fname)
        img_undistorted = cv2.remap(img, mapX, mapY, cv2.INTER_LINEAR)
        
        # Display
        cv2.imshow('undistorted image', img_undistorted)
        cv2.waitKey(0)

