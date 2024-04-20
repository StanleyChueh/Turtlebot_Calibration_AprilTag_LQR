import cv2
import numpy as np

def calibrate_camera(checkerboard_size=(9, 6), num_required_captures=20):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    object_points = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    object_points[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

    threed_points = []
    twod_points = []

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    count_captures = 0

    while count_captures < num_required_captures:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame. Exiting ...")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, 
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret:
            threed_points.append(object_points)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            twod_points.append(corners_refined)
            cv2.drawChessboardCorners(frame, checkerboard_size, corners_refined, ret)
            count_captures += 1

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(threed_points) > 0:
        ret, camera_matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(threed_points, twod_points, 
                                                                             gray.shape[::-1], None, None)
        if ret:
            return camera_matrix, distortion
        else:
            print("Calibration failed.")
            return None, None
    else:
        print("Not enough captures for calibration.")
        return None, None

def extract_camera_parameters(camera_matrix):
    if camera_matrix is not None:
        camera_fx = camera_matrix[0, 0]
        camera_fy = camera_matrix[1, 1]
        camera_cx = camera_matrix[0, 2]
        camera_cy = camera_matrix[1, 2]
        return camera_fx, camera_fy, camera_cx, camera_cy
    else:
        return None

# Calibrate camera
camera_matrix, dist_coeffs = calibrate_camera()
if camera_matrix is not None and dist_coeffs is not None:
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)

# Extract camera parameters
camera_params = extract_camera_parameters(camera_matrix)
if camera_params is not None:
    print("Camera parameters (fx, fy, cx, cy):", camera_params)
