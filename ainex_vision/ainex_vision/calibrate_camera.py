import cv2
import numpy as np
import glob
import yaml
import os
import sys

def main():
    # Parameter Setup
    CHECKERBOARD = (8, 6) # Number of inner corners of the checkerboard
    SQUARE_SIZE = 0.024

    pkg_path = os.path.dirname(__file__)
    img_path = os.path.join(pkg_path, "calib_imgs", "*.png")
    images = glob.glob(img_path)


    # Step 1: Generate 3D object points
    # Create a (Nx*Ny, 3) array of 3D points initialized to zero.
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    # Fill in the 2D grid coordinates such as (0,0), (1,0), ..., (8,5).
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)
    # Scale with the physical square size (meters)
    objp *= SQUARE_SIZE
    # Lists to store all object (3D) points and image (2D) points
    objpoints = []
    imgpoints = []

    # Step 2: Read all calibration images
    # Load all images from folder "calib_images"
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor (img, cv2.COLOR_BGR2GRAY)

        # Detect checkerborad corners
        ok, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ok:
            print("Detected corners in:", fname)

            # Append 3D object points
            objpoints.append(objp)
            # Refine corner locations to sub-pixel accuracy
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            # Append refined 2D image points
            imgpoints.append(corners2)
    print(f"\nDetected images: {len(imgpoints)} / {len(images)}")

    # Step 3: Perform camera calibration
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print("\nCamera Matrix:\n", K)
    print("\nDistortion Coefficients:\n", D)

    # Step 4: Compute reprojection error
    total_err = 0
    for i in range(len(objpoints)):
        # Reproject 3D points to 2D using rvec and tvec
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        # Compute difference to detect corners
        err = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_err += err

    print("\nMean Reprojection Error :", total_err / len(objpoints))

    # Step 5: Save undistorted test image
    test_img = cv2.imread(images[0])
    undistorted = cv2.undistort(test_img, K, D)

    cv2.imwrite(os.path.join(pkg_path, "raw.jpg"), test_img)
    cv2.imwrite(os.path.join(pkg_path, "undistorted.jpg"), undistorted)

    print("Saved raw.jpg and undistorted.jpg for report.")

    # Step 6: Save calibration to YAML

    yaml_data = {
        "image_width": test_img.shape[1],
        "image_height": test_img.shape[0],
        "camera_matrix": {"rows": 3, "cols": 3, "data": K.flatten().tolist()},
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": D.flatten().tolist()},
    }
    yaml_path = os.path.join(pkg_path, "camera.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(yaml_data, f)

    print("Calibration saved to camera.yaml")
    
if __name__ == "__main__":
    main()