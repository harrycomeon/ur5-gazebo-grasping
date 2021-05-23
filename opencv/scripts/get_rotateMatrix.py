#!/usr/bin/env python
################################################################################
#
# Introduction
##############
# This script generate a rotate matrix from Euler angle
# 
import numpy as np
import scipy.linalg as linalg
import math 
#variable is axis and radian
def rotate_mat(axis,radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix

if __name__ == "__main__":
    axis_x, axis_y, axis_z = [1,0,0], [0,1,0], [0, 0, 1]
    roll= -3.141
    pitch = 0 #pi/2
    yaw = -1.57
    rot_matrix_x = rotate_mat(axis_x, roll)#rotate x axis 
    rot_matrix_y = rotate_mat(axis_y, pitch)#rotate y axis 
    rot_matrix_z = rotate_mat(axis_z, yaw)#rotate z axis 
    print(rot_matrix_x)
    print(rot_matrix_y)
    print(rot_matrix_z)
    R = np.dot(rot_matrix_z, np.dot(rot_matrix_y,rot_matrix_x))
    print(R)
#[[ -2.03673204e-04   9.99999979e-01   0.00000000e+00]
 #[  9.99999804e-01   2.03673168e-04  -5.92653555e-04]
 #[ -5.92653543e-04  -1.20707648e-07  -9.99999824e-01]]
#[[ -2.03673204e-04   9.99999979e-01   0.00000000e+00]
 #[  9.99999804e-01   2.03673168e-04   5.92653555e-04]
 #[  5.92653543e-04   1.20707648e-07  -9.99999824e-01]]
#[[ -2.03673204e-04  -9.99999804e-01   5.92653543e-04]
 #[ -9.99999979e-01   2.03673168e-04  -1.20707648e-07]
 #[  0.00000000e+00  -5.92653555e-04  -9.99999824e-01]]


