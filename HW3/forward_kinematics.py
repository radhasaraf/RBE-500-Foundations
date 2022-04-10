import numpy as np
from math import pi, cos, sin, atan2, sqrt

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position and orientation of joint 4 (as the end-effector):
    # [x, y, z, roll, pitch, yaw]

    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]

    # Link lengths
    l1 = 0.1039
    l2 = 0.1581
    l3 = 0.150

    # DH Parameters table
    dh_table = np.matrix(
        [
            [pi + joint1, l1, 0, pi/2],
            [2*pi - atan2(3, 1) + joint2, 0, -l2, -pi],
            [pi - atan2(3, 1) + joint3, 0, l3, pi/2],
        ]
    )

    # Transformation matrices
    A1 = get_transformation_matrix(dh_table[0])
    A2 = get_transformation_matrix(dh_table[1])
    A3 = get_transformation_matrix(dh_table[2])

    # Homogenous transformation matrix
    H = np.matmul(A1, np.matmul(A2, A3))

    # Rotation matrix from homogenous transformation matrix
    r = np.matrix(
        [
            [H[0, 0], H[0, 1], H[0, 2]],
            [H[1, 0], H[1, 1], H[1, 2]],
            [H[2, 0], H[2, 1], H[2, 2]]
        ]
    )
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]

    # convert the final orientation of the end-effector to roll-pitch-yaw
    roll = atan2(r[2, 1], r[2, 2])
    pitch = atan2(-r[2, 0], sqrt(r[2, 1]*r[2, 1] + r[2, 2]*r[2, 2]))
    yaw = atan2(r[1, 0], r[0, 0])

    return [x, y, z, roll, pitch, yaw]


def get_transformation_matrix(dh_row):
    theta, d, a, alpha = dh_row[0, 0], dh_row[0, 1], dh_row[0, 2], dh_row[0, 3]

    return np.matrix(
        [
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ]
    )