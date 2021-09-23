"""!
Implements Forward and backwards kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
# from scipy.linalg import expm

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    H = np.eye(4)
    for i in range(link):
        #First convert incoming joint angles to radians
        dh_params[i][3] = dh_params[i][3] + joint_angles[i]
        clamp(dh_params[i][3])
        T_curr = get_transform_from_dh(dh_params[i][0], dh_params[i][1], dh_params[i][2], dh_params[i][3])
        H = H @ T_curr
        # print(f"link {i+1}: {T_curr}")
        # print(f"{i}: {dh_params[i]}")
    return H

def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """
    T = np.zeros([4,4])
    T[0,:] = [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)]
    T[1,:] = [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)]
    T[2,:] = [0, np.sin(alpha), np.cos(alpha), d]
    T[3,:] = [0,0,0,1]
    return T

def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    #ZYZ parameterization
    # print(f"{T}")
    r33 = T[2,2]
    r13 = T[0,2]
    r23 = T[1,2]
    r32 = T[2,1]
    r31 = T[2,0]
    euler = np.zeros([3,1])
    # euler[1] = np.arctan2(np.sqrt(T[2,0]**2 + T[2,1]**2) , T[2,2])
    # euler[0] = np.arctan2(T[1,2] / np.sin(euler[1]) , T[0,2] / np.sin(euler[1]))
    # euler[2] = np.arctan2(T[2,1] / np.sin(euler[1]) , T[2,0] / np.sin(euler[1]))
    euler[1] = np.arctan2(np.sqrt(1 - r33**2) , r33) #theta
    euler[0] = np.arctan2(r23, r13) #phi
    euler[2] = np.arctan2(r32, -r31)
    # euler[2] = clamp(np.arctan2(r32, -r31) + np.pi) #psi
    return euler

def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,phi) where phi is
                rotation about base frame y-axis

    @param      T     transformation matrix

    @return     The pose from T.
    """
    pose = np.zeros([6,1])
    euler = get_euler_angles_from_T(T)
    pose = [T[0,3], T[1,3], T[2,3], euler[0], euler[1], euler[2]]
    # print(f"phi:{euler[0]}\ntheta:{euler[1]}\npsi:{euler[2]}")
    return pose


def FK_pox(joint_angles):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    return np.array([0, 0, 0, 0])

def to_s_matrix(w,v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    #Variables
    x = pose[0]
    y = pose[1]
    z = pose[2]
    # phi = pose[3]
    theta = pose[4]
    psi = pose[5]
    theta5 = clamp(psi + np.pi)
    tool_angle = np.pi / 2 - theta #Different than theta4
    #Constants
    l1 = dh_params[0][2]
    l2 = dh_params[1][0]
    l3 = dh_params[2][0]
    l4 = dh_params[4][2]

    #Inverse kinematics - Has 4 possible ways of reaching a particular point
    """Forward"""
    theta1_fwd  = np.arctan2(y,x)
    a1 = x - np.sign(x)* abs(l4*np.cos(theta1_fwd)*np.cos(tool_angle))
    b1 = y - np.sign(y)* abs(l4*np.sin(theta1_fwd)*np.cos(tool_angle))
    r           = np.sqrt(a1**2 + b1**2)
    zeff        = z - l1 - l4*np.sin(tool_angle)
    A           = np.sqrt(zeff**2 + r**2)
    # print(f"{theta1_fwd}, {zeff}, {A}, {l2}, {l3}")
    theta3_temp  = np.arccos((A**2 - l2**2 - l3**2)/(2*l2*l3))
    a           = l3*np.sin(theta3_temp)
    b           = l2 + l3*np.cos(theta3_temp)

    theta3_1  = np.pi/2 - 0.38 - theta3_temp
    theta3_2  = np.pi/2 - 0.38 + theta3_temp

    theta2_fwd_down    = np.arctan2(zeff, r) - np.arctan2(a,b) + 0.38 - np.pi/2
    theta2_fwd_up    = np.arctan2(zeff,r) + np.arctan2(a,b) + 0.38 - np.pi/2
    theta4_fwd_down    = tool_angle - (theta2_fwd_down + theta3_2)
    theta4_fwd_up    = tool_angle - (theta2_fwd_up + theta3_1)

    """Backward"""
    theta1_bkwd  = np.pi + np.arctan2(y,x)
    a1 = x - np.sign(x)* abs(l4*np.cos(theta1_bkwd)*np.cos(tool_angle))
    b1 = y - np.sign(y)* abs(l4*np.sin(theta1_bkwd)*np.cos(tool_angle))
    r           = np.sqrt(a1**2 + b1**2)
    zeff        = z - l1 - l4*np.sin(tool_angle)
    A           = np.sqrt(zeff**2 + r**2)
    # print(f"theta1_bkwd:{theta1_bkwd}\na1:{a1}\nb1:{b1}\nr:{r}\nzeff:{zeff}\nA:{A}")
    a           = l3*np.sin(theta3_temp)
    b           = l2 + l3*np.cos(theta3_temp)
    # print(f"{(A**2 - l2**2 - l3**2)/(2*l2*l3)}")

    theta2_bkwd_up    = np.pi/2 - (np.arctan2(zeff, r) + np.arctan2(a,b)) + 0.38
    theta2_bkwd_down    = np.pi/2 + 0.38 - (np.arctan2(zeff,r) - np.arctan2(a,b))
    theta4_bkwd_up    = tool_angle - (theta2_bkwd_up + theta3_2) + np.pi
    theta4_bkwd_down    = tool_angle - (theta2_bkwd_down + theta3_1) + np.pi

    """Pack all 4 possible answers into a numpy matrix"""
    joints = np.zeros((4,5))
    joints[0,:] = [theta1_fwd, theta2_fwd_down, theta3_2, theta4_fwd_down, theta5] #fwd_down
    joints[1,:] = [theta1_fwd, theta2_fwd_up, theta3_1, theta4_fwd_up, theta5] #fwd_up
    joints[2,:] = [theta1_bkwd, theta2_bkwd_down, theta3_1, theta4_bkwd_down, theta5] #bkwd_down
    joints[3,:] = [theta1_bkwd, theta2_bkwd_up, theta3_2, theta4_bkwd_up, theta5] #bkwd_up

    return joints
