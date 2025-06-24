import cv2
import numpy as np
import glob

# Define the dimensions of checkerboard 
CHECKERBOARD = (6, 9)

# stop the iteration when specified 
# accuracy, epsilon, is reached or 
# specified number of iterations are completed. 
criteria = (cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Vector for 3D points 
threedpoints = []

# Vector for 2D points 
twodpoints = []

# 3D points real world coordinates 
objectp3d = np.zeros((1, CHECKERBOARD[0] 
                    * CHECKERBOARD[1], 
                    3), np.float32) 
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 
                            0:CHECKERBOARD[1]].T.reshape(-1, 2) 
prev_img_shape = None

# Extracting path of individual image stored 
# in a given directory. Since no path is 
# specified, it will take current directory 
# jpg files alone 
# images = glob.glob('C:\\Users\\ADLAB002\\Desktop\\Sun_sensor_ws\\calibrate_image_P1\\*.jpg') #camera of p pech
images = glob.glob('*.jpg') # my camera 
q
for filename in images: 
    image = cv2.imread(filename) 
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners 
    # If desired number of corners are 
    # found in the image then ret = true 
    ret, corners = cv2.findChessboardCorners( 
                    grayColor, CHECKERBOARD, 
                    cv2.CALIB_CB_ADAPTIVE_THRESH 
                    + cv2.CALIB_CB_FAST_CHECK +
                    cv2.CALIB_CB_NORMALIZE_IMAGE) 

    # If desired number of corners can be detected then, 
    # refine the pixel coordinates and display 
    # them on the images of checker board 
    if ret == True: 
        threedpoints.append(objectp3d)

        # Refining pixel coordinates 
        # for given 2d points. 
        corners2 = cv2.cornerSubPix( 
            grayColor, corners, (11, 11), (-1, -1), criteria)

        twodpoints.append(corners2)

        # Draw and display the corners 
        image = cv2.drawChessboardCorners(image, 
                                        CHECKERBOARD, 
                                        corners2, ret)

    cv2.imshow('img', image) 
    cv2.waitKey(0) 

cv2.destroyAllWindows() 

h, w = image.shape[:2] 

# Perform camera calibration by 
# passing the value of above found out 3D points (threedpoints) 
# and its corresponding pixel coordinates of the 
# detected corners (twodpoints) 
ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera( 
    threedpoints, twodpoints, grayColor.shape[::-1], None, None)

# Displaying required output 
print("Camera matrix:")
print(matrix)

print("\nDistortion coefficient:")
print(distortion)

print("\nRotation Vectors:")
print(r_vecs)

print("\nTranslation Vectors:")
print(t_vecs)

# Save the calibration data
calibration_data = {
    "camera_matrix": matrix,
    "distortion_coefficients": distortion,
    "rotation_vectors": r_vecs,
    "translation_vectors": t_vecs
}

# Save data to a .npz file
# np.savez("calibration_data_2.npz", **calibration_data)
# print("\nCalibration data has been saved to 'calibration_data_2.npz'")
np.savez("calibration_data.npz", **calibration_data)
print("\nCalibration data has been saved to 'calibration_data.npz'")

# Function to calculate Reprojection Error
def calculate_reprojection_error(object_points, image_points, rvecs, tvecs, camera_matrix, distortion_coefficients):
    """
    คำนวณ Reprojection Error โดยการเปรียบเทียบจุดที่คาดว่าจะทำนายได้จากกล้อง
    กับจุดที่ได้จากการจับภาพจริง
    """
    total_error = 0
    total_points = 0

    # Loop ผ่านทุกชุดของ object_points และ image_points
    for obj_pts, img_pts, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        # ทำการทำนายจุด 2D จากพิกัด 3D ด้วย cv2.projectPoints
        img_points_reprojected, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, distortion_coefficients)

        # คำนวณความแตกต่างระหว่างจุดจริงและจุดที่คาดการณ์ได้
        error = cv2.norm(img_pts, img_points_reprojected, cv2.NORM_L2) / len(img_pts)
        total_error += error
        total_points += 1

    # คำนวณค่าเฉลี่ยของ Reprojection Error
    mean_error = total_error / total_points
    return mean_error


# คำนวณ Reprojection Error
reprojection_error = calculate_reprojection_error(threedpoints, twodpoints, r_vecs, t_vecs, matrix, distortion)

# แสดงผล Reprojection Error
print(f"Reprojection Error: {reprojection_error} pixels")
# Extract fx and fy from the intrinsic matrix
fx = matrix[0, 0]
fy = matrix[1, 1]

# Width and height of the image
# Already defined as w, h from earlier:
# h, w = image.shape[:2]

# Calculate HFOV and VFOV in degrees
HFOV = 2 * np.arctan(w / (2 * fx)) * 180 / np.pi
VFOV = 2 * np.arctan(h / (2 * fy)) * 180 / np.pi

print(f"\nHFOV: {HFOV:.2f} degrees")
print(f"VFOV: {VFOV:.2f} degrees")