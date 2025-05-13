import cv2
import numpy as np
import os
import glob
from scipy.spatial.transform import Rotation as R

# Configuration
chessboard_size = (9, 7)
square_size = 20.0 
image_folder = r'C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\camera_calibration_images_old'
pose_file = r'C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\robot_coordinates_calibration.txt'

# Generate 3D points for chessboard
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Chessboard to TCP transform from the Fanuc Pendant
T_chessboard2tcp = np.eye(4)
T_chessboard2tcp[:3, 3] = [-1.927, -60.004, 86.016]

# TCP to base form fanuc using FRC_ReadCartesianPosition
R_gripper2base = []
t_gripper2base = []

with open(pose_file, 'r') as f:
    robot_lines = f.readlines()

image_paths = sorted(glob.glob(os.path.join(image_folder, 'calib_image_*.png')))
print(f"Found {len(image_paths)} images.")

# Positions w.r.t camera
R_target2cam = []
t_target2cam = []

for i, image_path in enumerate(image_paths):
    if i >= len(robot_lines):
        print(f"Skipping image {image_path} due to missing robot pose.")
        continue

    img = cv2.imread(image_path)
    # skip images where no chessboard is detected by opencv
    if img is None:
        print(f"Image not found or unreadable: {image_path}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # skip images where no chessboard is detected by opencv
    if not ret:
        print(f"Chessboard not found in: {image_path}")
        continue
    else:
        print(f"Chessboard detected in: {image_path}")

    corners_refined = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )

    # Camera intrinsics (replace with your calibrated values if available)
    h, w = img.shape[:2]
    camera_matrix = camera_matrix = np.array([
    [883.7183045, 0, 631.43111795],
    [0, 885.02503402, 370.4941652],
    [0, 0, 1]], dtype=np.float64
    )
    
    dist_coeffs = np.array([
    -1.73560396e-02,
     9.33949648e-01,
    -2.38302311e-03,
    -2.46210736e-03,
    -3.48952505e+00
    ])

    success, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)
    # skip images where no chessboard is detected by opencv
    if not success:
        print(f"solvePnP failed for {image_path}")
        continue

    R_cam, _ = cv2.Rodrigues(rvec)
    try:
        #adding robot pose where chessboard detected to list
        x, y, z, w, p, r_deg = map(float, robot_lines[i].strip().split(','))
        T_tcp2base = np.eye(4)
        T_tcp2base[:3, :3] = R.from_euler('ZYX', [w, p, r_deg], degrees=True).as_matrix()
        T_tcp2base[:3, 3] = [x, y, z]
        T_chessboard2base = T_tcp2base @ T_chessboard2tcp
        R_gripper2base.append(T_chessboard2base[:3, :3])
        t_gripper2base.append(T_chessboard2base[:3, 3].reshape(3, 1))
    except Exception as e:
        print(f"Skipping pose for {image_path}: {e}")
        continue
    R_target2cam.append(R_cam)
    t_target2cam.append(tvec)

print(f"Used {len(R_gripper2base)} valid image-pose pairs out of {len(image_paths)}")

# Run hand-eye calibration
R_cam2base, t_cam2base = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

print("Camera-to-Base Transform:")
print("Rotation:\n", R_cam2base)
print("Translation:\n", t_cam2base)

# Testing  Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing   Testing 

image_path = r'C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\calib_image_27.png'
pose_file = r'C:\Users\draval\OneDrive - Bishop-Wisecarver Corporation\Desktop\FanucStuff\Code\VaccinePython\Project\utils\test.txt'

# Chessboard
chessboard_size = (9, 7)
square_size = 20.0
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Load robot pose
with open(pose_file, 'r') as f:
    x, y, z, w, p, r_deg = map(float, f.readline().strip().split(','))
    T_tcp2base = np.eye(4)
    T_tcp2base[:3, :3] = R.from_euler('ZYX', [w, p, r_deg], degrees=True).as_matrix()
    T_tcp2base[:3, 3] = [x, y, z]

# Apply offset to get actual chessboard pose
T_chessboard2base = T_tcp2base @ T_chessboard2tcp
T_tcp2base = T_chessboard2base  # use this corrected value

# Convert camera-to-base to base-to-camera
T_base2cam = np.eye(4)
T_base2cam[:3, :3] = R_cam2base.T
T_base2cam[:3, 3] = (-R_cam2base.T @ t_cam2base).flatten()

# chessboard in camera frame
T_tcp2cam = T_base2cam @ T_tcp2base
rvec_pred, _ = cv2.Rodrigues(T_tcp2cam[:3, :3])
tvec_pred = T_tcp2cam[:3, 3].reshape(3, 1)

# Image observation
img = cv2.imread(image_path)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

if not ret:
    print("Chessboard not found in test image.")
    exit()

corners_refined = cv2.cornerSubPix(
    gray, corners, (11, 11), (-1, -1),
    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
)

# Camera intrinsics
h, w = img.shape[:2]
camera_matrix = np.array([
    [883.7183045, 0, 631.43111795],
    [0, 885.02503402, 370.4941652],
    [0, 0, 1]], dtype=np.float64
)
dist_coeffs = np.array([
    -1.73560396e-02,
     9.33949648e-01,
    -2.38302311e-03,
    -2.46210736e-03,
    -3.48952505e+00
])

# Actual camera observation from image
success, rvec_obs, tvec_obs = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)

print("Predicted chessboard pose in camera frame:")
print("rvec (Rodrigues):\n", rvec_pred.flatten())
print("tvec:\n", tvec_pred.flatten())

print("\nObserved chessboard pose from image (solvePnP):")
print("rvec (Rodrigues):\n", rvec_obs.flatten())
print("tvec:\n", tvec_obs.flatten())


rot_error = np.linalg.norm(rvec_pred - rvec_obs)
trans_error = np.linalg.norm(tvec_pred - tvec_obs)
print(f"\nRotation error (Rodrigues norm): {rot_error:.3f}")
print(f"Translation error (mm): {trans_error:.3f}")