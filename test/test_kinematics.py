#!/usr/bin/python3
"""!
Test kinematics

TODO: Use this file and modify as you see fit to test kinematics.py
"""

import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
from kinematics import *
from copy import deepcopy

passed = True
vclamp = np.vectorize(clamp)

dh_params = [[0,1.57,0.12,0.0],
             [0.102,0,0,1.19],
             [0.1,0,0,-1.19],
             [0,1.57,0,1.57],
             [0,0,0.11,0]]

# fk_angles = [
#     [0.0,           0.0,            0.0,            0.0 ,  0.0],
#     [np.pi * 0.1,   0.0,            np.pi / 2,      0.0, 0.0],
#     [np.pi * 0.25,  np.pi / 2,      -np.pi / 2,     np.pi / 2, 0.0],
#     [np.pi * 0.4,   np.pi / 2,      -np.pi / 2,     0.0, 0.0],
#     [np.pi * 0.55,  0,              0,              0, 0.0],
#     [np.pi * 0.7,   0.0,            np.pi / 2,      0.0, 0.0],
#     [np.pi * 0.85,  np.pi / 2,      -np.pi / 2,     np.pi / 2, 0.0],
#     [np.pi,         np.pi / 2,      -np.pi / 2,     0.0, 0.0],
#     [0.0,           np.pi / 2,      np.pi / 2,      0.0, 0.0],
#     [np.pi / 2,     -np.pi / 2,     np.pi / 2,      0.0, 0.0]]

fk_angles = [[0, -0.0,            0.0,            0.0,   1],]

print('Test FK')
fk_poses = []
for joint_angles in fk_angles:
    print('Joint angles:', joint_angles)
    for i in range(len(joint_angles) + 1):
        pose = get_pose_from_T(FK_dh(deepcopy(dh_params), joint_angles, i))
        print('Link {} pose: {}'.format(i, pose))
        if i == len(joint_angles):
            fk_poses.append(pose)
    print(fk_poses)

print('Test IK')
for pose, angles in zip(fk_poses, fk_angles):
   matching_angles = False
   print('Pose: {}'.format(pose))
   options = IK_geometric(deepcopy(dh_params), pose)
   for i, joint_angles in enumerate(options):
       print('Option {}: {}'.format(i, joint_angles))
       compare = vclamp(joint_angles - angles)
       if np.allclose(compare, np.zeros_like(compare), rtol=5e-2, atol=5e-2):
           print('Option {} matches angles used in FK'.format(i))
           matching_angles = True
   if not matching_angles:
       print('No match to the FK angles found!')
       passed = False
   print()

# print('Test options')
# fk_poses = []
# for joint_angles in options:
#   print('Joint angles:', joint_angles)
#   for i in range(len(joint_angles) + 1):
#       pose = get_pose_from_T(FK_dh(deepcopy(dh_params), joint_angles, i))
#       # print('Link {} pose: {}'.format(i, pose))
#       if i == len(joint_angles):
#           fk_poses.append(pose)
#           print(pose)
