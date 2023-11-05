import cv2 as cv
import glob
import numpy as np
import matplotlib.pyplot as plt

ROWS = 6
COLUMNS = 9


class StereoCameraSystem:
    def __init__(self, right_dir, left_dir, sync_dir, checkerborad_size, load_dir):
        self.COLUMNS, self.ROWS = checkerborad_size
        self.left_dir = left_dir
        self.right_dir = right_dir
        self.sync_dir = sync_dir
        if load_dir != None:
            l = np.load(load_dir)
            self.R = l['R']
            self.T = l['T']

    def calibrate(self, save_dir):
        self.mtxr, distr = self.calibrate_single_camera(self.right_dir)
        self.mtxl, distl = self.calibrate_single_camera(self.left_dir)
        self.R, self.T = self.stereo_calibrate(
            self.mtxr, distr, self.mtxl, distl, self.sync_dir)
        if save_dir != None:
            np.savez(save_dir, R=self.R, T=self.T)

    def calibrate_single_camera(self, images_folder):
        images_names = glob.glob(images_folder)
        images = []
        for imname in images_names:
            im = cv.imread(imname, 1)
            images.append(im)

        # criteria used by checkerboard pattern detector.
        # Change this if the code can't find the checkerboard
        criteria = (cv.TERM_CRITERIA_EPS +
                    cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        world_scaling = 1.  # change this to the real world square size. Or not.

        # coordinates of squares in the checkerboard world space
        objp = np.zeros((self.ROWS*self.COLUMNS, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.ROWS, 0:self.COLUMNS].T.reshape(-1, 2)
        objp = world_scaling * objp

        # frame dimensions. Frames should be the same size.
        width = images[0].shape[1]
        height = images[0].shape[0]

        # Pixel coordinates of checkerboards
        imgpoints = []  # 2d points in image plane.

        # coordinates of the checkerboard in checkerboard world space.
        objpoints = []  # 3d point in real world space

        for frame in images:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # find the checkerboard
            ret, corners = cv.findChessboardCorners(
                gray, (self.ROWS, self.COLUMNS), None)
            if ret == True:

                # Convolution size used to improve corner detection. Don't make this too large.
                conv_size = (11, 11)

                # opencv can attempt to improve the checkerboard coordinates
                corners = cv.cornerSubPix(
                    gray, corners, conv_size, (-1, -1), criteria)
                cv.drawChessboardCorners(
                    frame, (self.ROWS, self.COLUMNS), corners, ret)
                cv.imshow('img', frame)
                cv.waitKey(500)

                objpoints.append(objp)
                imgpoints.append(corners)

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints, imgpoints, (width, height), None, None)
        print('rmse:', ret)
        print('camera matrix:\n', mtx)
        print('distortion coeffs:', dist)
        print('Rs:\n', rvecs)
        print('Ts:\n', tvecs)

        return mtx, dist

    def stereo_calibrate(self, mtx1, dist1, mtx2, dist2, frames_folder):
        # read the synched frames
        images_names = glob.glob(frames_folder)
        images_names = sorted(images_names)

        c1_images_names = images_names[:len(images_names)//2]
        c2_images_names = images_names[len(images_names)//2:]

        print(c1_images_names)
        c1_images = []
        c2_images = []
        for im1, im2 in zip(c1_images_names, c2_images_names):
            _im = cv.imread(im1, 1)
            c1_images.append(_im)

            _im = cv.imread(im2, 1)
            c2_images.append(_im)

        # change this if stereo calibration not good.
        criteria = (cv.TERM_CRITERIA_EPS +
                    cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

        world_scaling = 1.  # change this to the real world square size. Or not.

        # coordinates of squares in the checkerboard world space
        objp = np.zeros((self.ROWS*self.COLUMNS, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.ROWS, 0:self.COLUMNS].T.reshape(-1, 2)
        objp = world_scaling * objp

        # frame dimensions. Frames should be the same size.
        width = c1_images[0].shape[1]
        height = c1_images[0].shape[0]

        # Pixel coordinates of checkerboards
        imgpoints_left = []  # 2d points in image plane.
        imgpoints_right = []

        # coordinates of the checkerboard in checkerboard world space.
        objpoints = []  # 3d point in real world space

        for frame1, frame2 in zip(c1_images, c2_images):
            size = (self.ROWS, self.COLUMNS)
            gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
            gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
            c_ret1, corners1 = cv.findChessboardCorners(gray1, size, None)
            c_ret2, corners2 = cv.findChessboardCorners(gray2, size, None)

            print(len(corners1))
            print(len(corners2))

            if c_ret1 == True and c_ret2 == True:
                corners1 = cv.cornerSubPix(
                    gray1, corners1, (11, 11), (-1, -1), criteria)
                corners2 = cv.cornerSubPix(
                    gray2, corners2, (11, 11), (-1, -1), criteria)

                cv.drawChessboardCorners(frame1, size, corners1, c_ret1)
                cv.imshow('left', frame1)

                cv.drawChessboardCorners(frame2, size, corners2, c_ret2)
                cv.imshow('right', frame2)
                cv.waitKey(500)

                objpoints.append(objp)
                imgpoints_left.append(corners1)
                imgpoints_right.append(corners2)

        stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
        ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx1, dist1,
                                                                     mtx2, dist2, (width, height), criteria=criteria, flags=stereocalibration_flags)

        print(ret)
        return R, T

    def triangulate(self, right_points, left_points):
        # RT matrix for C1 is identity.
        RT1 = np.concatenate([np.eye(3), [[0], [0], [0]]], axis=-1)
        P1 = self.mtxr @ RT1  # projection matrix for C1

        # RT matrix for C2 is the R and T obtained from stereo calibration.
        RT2 = np.concatenate([self.R, self.T], axis=-1)
        P2 = self.mtxl @ RT2  # projection matrix for C2

        def DLT(P1, P2, point1, point2):

            A = [point1[1]*P1[2, :] - P1[1, :],
                 P1[0, :] - point1[0]*P1[2, :],
                 point2[1]*P2[2, :] - P2[1, :],
                 P2[0, :] - point2[0]*P2[2, :]
                 ]
            A = np.array(A).reshape((4, 4))
            # print('A: ')
            # print(A)

            B = A.transpose() @ A
            from scipy import linalg
            U, s, Vh = linalg.svd(B, full_matrices=False)

            print('Triangulated point: ')
            print(Vh[3, 0:3]/Vh[3, 3])
            return Vh[3, 0:3]/Vh[3, 3]

        p3ds = []
        for right_point, left_point in zip(right_point, left_points):
            _p3d = DLT(P1, P2, right_point, left_point)
            p3ds.append(_p3d)
        p3ds = np.array(p3ds)

        return p3ds
