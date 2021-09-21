#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                    [-0.7040, 0.7102, -0.0053],
                    [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)
    g0 = np.array([[R[0,0], R[0,1], R[0,2],qs[0,7]],
                       [R[1,0], R[1,1], R[1,2],qs[1,7]],
                       [R[2,0], R[2,1], R[2,2],qs[2,7]],
                       [0, 0, 0, 1]])

    v1 = -np.cross(ws[0:3,0], qs[0:3,0])
    xi1 = np.array([[v1[0], v1[1], v1[2], ws[0,0], ws[1,0], ws[2,0]]]).T
    g1 = kfs.homog_3d(xi1, joint_angles[0,0])

    v2 = -np.cross(ws[0:3,1], qs[0:3,1])
    xi2 = np.array([[v2[0,0], v2[1,0], v2[2,0], ws[0,1], ws[1,1], ws[2,1]]]).T
    g2 = kfs.homog_3d(xi2, joint_angles[1,0])

    v3 = -np.cross(ws[0:3,2], qs[0:3,2])
    xi3 = np.array([[v3[0,0], v3[1,0], v3[2,0], ws[0,2], ws[1,2], ws[2,2]]]).T
    g3 = kfs.homog_3d(xi3, joint_angles[2,0])

    v4 = -np.cross(ws[0:3,3], qs[0:3,3])
    xi4 = np.array([[v4[0,0], v4[1,0], v4[2,0], ws[0,3], ws[1,3], ws[2,3]]]).T
    g4 = kfs.homog_3d(xi4, joint_angles[3,0])

    v5 = -np.cross(ws[0:3,4], qs[0:3,4])
    xi5 = np.array([[v5[0,0], v5[1,0], v5[2,0], ws[0,4], ws[1,4], ws[2,4]]]).T
    g5 = kfs.homog_3d(xi5, joint_angles[4,0])

    v6 = -np.cross(ws[0:3,5], qs[0:3,5])
    xi6 = np.array([[v6[0,0], v6[1,0], v6[2,0], ws[0,5], ws[1,5], ws[2,5]]]).T
    g6 = kfs.homog_3d(xi6, joint_angles[5,0])

    v7 = -np.cross(ws[0:3,6], qs[0:3,6])
    xi7 = np.array([[v7[0,0], v7[1,0], v7[2,0], ws[0,6], ws[1,6], ws[2,6]]]).T
    g7 = kfs.homog_3d(xi7, joint_angles[6,0])


    g_st = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(g1, g2), g3), g4), g5), g6), g7), g0)

    return g_st


def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    # angles[0] = joint_state[2]
    # angles[1] = joint_state[3]
    # angles[2] = joint_state[4]
    # angles[3] = joint_state[5]
    # angles[4] = joint_state[6]
    # angles[5] = joint_state[7]
    # angles[6] = joint_state[8]
    print(joint_state)
    
    g_st = baxter_forward_kinematics_from_angles(angles)

    print(baxter_forward_kinematics_from_angles(angles))

    return g_st
