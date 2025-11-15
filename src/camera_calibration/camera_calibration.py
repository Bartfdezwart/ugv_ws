import argparse
from pathlib import Path
import numpy as np
import cv2 as cv2
import yaml


def main(args):
    chessboard_size = (8, 6)
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : chessboard_size[0], 0 : chessboard_size[1]].T.reshape(
        -1, 2
    )

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    images_with_corners_found = []

    images = args.img_dir.glob("*.png")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        # If found, add object points, image points (after refining them)
        if ret:
            images_with_corners_found.append(fname)

            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            if args.show_img:
                cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
                cv2.imshow("img", img)
                cv2.waitKey(0)

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    if args.action == "calibrate" or args.action == "error":
        n_images = sum(1 for _ in args.img_dir.glob("*.png"))
        print(f"Succesful: {len(images_with_corners_found)}/{n_images}")

    if args.action == "calibrate":
        print(f"RMS re-projection error: {ret}")
    elif args.action == "error":
        with args.param_file.open(mode="r") as file:
            config = yaml.safe_load(file)
            cam_mtx = config["camera_matrix"]
            mtx = np.array(cam_mtx["data"]).reshape(
                (cam_mtx["rows"], cam_mtx["cols"])
            )
            dist = np.array(config["distortion_coefficients"]["data"])
            print("Using camera matrix and dist coeff from file")

        if args.no_dist_coef:
            dist = np.zeros(5)
            print("Overwrite dist coeff with zeros")

    print(f"Camera matrix:\n{mtx}")
    print(f"Distortion coefficients: {dist}")

    if args.action in ["calibrate", "error"]:
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(
                objpoints[i], rvecs[i], tvecs[i], mtx, dist
            )
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error

        print("MSE re-projection error: {}".format(mean_error / len(objpoints)))

    # Save camera matrix and distortion coefficients to a file
    if args.action == "calibrate" and args.output_file:
        data = {
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": mtx.flatten().tolist(),
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": dist.flatten().tolist(),
            },
        }
        if args.output_file.exists():
            raise FileExistsError(f"File already exists at `{args.output_file}`")
        with args.output_file.open(mode="w") as file:
            yaml.dump(data, file, sort_keys=False)
        print(
            f"Written camera matrix and distortion coefficients to `{args.output_file}`"
        )

    ################################################
    if args.action in ["calibrate", "error"]:
        # When no test_img is specified stop the program
        if args.test_img is None:
            return
        image_idx = args.test_img
        test_image = images_with_corners_found[image_idx]
        print(f"Test image: `{test_image.absolute()}`")
    else:
        raise NotImplementedError

    rvec = rvecs[image_idx]
    tvec = tvecs[image_idx]

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    T = np.hstack((rotation_matrix, tvec))

    correct_imgpoints = imgpoints[image_idx]
    proj_imgpoints = []
    for i, _ in enumerate(objp):
        point = objp[i : i + 1].T
        point = np.vstack((point, [1]))
        point = mtx @ T @ point
        point = (point[:2] / point[2]).flatten()

        proj_imgpoints.append(point.astype(int))

    image = cv2.imread(test_image)
    image_undistort = cv2.undistort(image, mtx, dist)
    image_undistort_with_proj = np.copy(image_undistort)
    for proj_point, chess_points in zip(proj_imgpoints, correct_imgpoints):
        # Draw true chess corner points
        cv2.circle(image, chess_points.flatten().astype(int), 4, (0, 255, 0), -1)
        undistorted_chess_points = cv2.undistortImagePoints(
            chess_points.astype(float), mtx, dist
        )

        # Draw the undistorted chessboard corners on the undistorted image
        undistorted_chess_points = undistorted_chess_points.flatten().astype(int)
        cv2.circle(
            image_undistort,
            undistorted_chess_points,
            4,
            (0, 255, 0),
            -1,
        )
        # Draw true chess corner points and the projected points
        cv2.circle(image_undistort_with_proj, tuple(proj_point), 6, (0, 0, 255), -1)
        cv2.circle(
            image_undistort_with_proj,
            undistorted_chess_points,
            4,
            (0, 255, 0),
            -1,
        )

    image = np.hstack((image, image_undistort))
    # Compute the pad
    pad_total = np.array(image.shape) - np.array(image_undistort_with_proj.shape)
    # Split pad so image is centered
    pad_size = [
        (pad_total[i] // 2, pad_total[i] - pad_total[i] // 2)
        for i in range(len(pad_total))
    ]
    image = np.vstack((image, np.pad(image_undistort_with_proj, pad_size)))
    cv2.imshow("Original image, Undistorted image, \n Projection", image)
    cv2.waitKey(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(
        title="action", dest="action", description="Action to perform", required=True
    )

    # Subcommand for camera calibration
    parser_calibrate = subparsers.add_parser(
        "calibrate", help="Perform camera calibration"
    )
    parser_calibrate.add_argument(
        "--img-dir",
        type=Path,
        help="The path to the directory where images are stored",
    )
    parser_calibrate.add_argument(
        "--output-file",
        type=Path,
        help="The file path of the output. Expected to be a yaml",
    )

    parser_calibrate.add_argument("--show-img", action="store_true")

    parser_calibrate.add_argument("--test-img", type=int)

    # Subcommand for computing the re-projection error on a set of images
    parser_error = subparsers.add_parser(
        "error",
        help="Compute the re-projection error on a set of images",
    )
    parser_error.add_argument(
        "--img-dir",
        type=Path,
        required=True,
        help="The path to the directory where images are stored",
    )
    parser_error.add_argument("--show-img", action="store_true")
    parser_error.add_argument("--test-img", type=int)
    parser_error.add_argument(
        "--param-file",
        type=Path,
        help="Path to a parameter file (yaml) that has the camera matrix and distortion coefficients",
    )
    parser_error.add_argument(
        "--no-dist-coef",
        action="store_true",
    )

    parsed_args = parser.parse_args()
    main(parsed_args)
