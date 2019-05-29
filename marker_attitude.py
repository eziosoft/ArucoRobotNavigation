#   This file is part of ArucoRobotNavigation.
#
#      ArucoRobotNavigation is free software: you can redistribute it and/or modify
#      it under the terms of the GNU General Public License as published by
#      the Free Software Foundation, either version 3 of the License, or
#      (at your option) any later version.
#
#      Foobar is distributed in the hope that it will be useful,
#      but WITHOUT ANY WARRANTY; without even the implied warranty of
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#      GNU General Public License for more details.
#
#      You should have received a copy of the GNU General Public License
#      along with Foobar.  If not, see <https://www.gnu.org/licenses/>.
#  Copyright (c) 2019.

import cv2
import cv2.aruco as aruco
import numpy as np
import math

# --- Get the camera calibration path
calib_path = ""
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])



def getMarkerAttitude(corners, real_marker_size):
    np_corners = np.array([[[corners[0]['x'], corners[0]['y']],
                            [corners[1]['x'], corners[1]['y']],
                            [corners[2]['x'], corners[2]['y']],
                            [corners[3]['x'], corners[3]['y']],
                            ]])
    #
    markerPose = aruco.estimatePoseSingleMarkers(np_corners, real_marker_size, camera_matrix, camera_distortion)
    rvec, tvec = markerPose[0][0, 0, :], markerPose[1][0, 0, :]
    xm = tvec[0]
    ym = tvec[1]
    zm = tvec[2]
    str_position = "marker position from camera [cm] x=%4.0f  y=%4.0f  z=%4.0f" % (xm, ym, zm)

    # -- Obtain the rotation matrix tag->camera
    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
    R_tc = R_ct.T

    # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)
    str_position += " roll=%4.0f  pitch=%4.0f  yaw=%4.0f" % (math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker))
    print(str_position)
    return (xm,ym,zm, roll_marker, pitch_marker,yaw_marker)
