#!/usr/bin/env python
import numpy as np
import scipy.linalg as linalg
import math
def pixelcoord2worldcoord(K,matrix_ex,pixel):
    slice=np.mat([0,0,0])
    splice = np.zeros((3,1))#create 3x1 zero matrix
    pixel_T = pixel.transpose()
    print("pixel_T:")
    print(pixel_T)
    K = np.hstack((K,splice))
    print("the camera intrinsic matrix value after splicing")
    print(K)
    # matrix is dot of intrinstic and external parameter
    matrix = np.dot(K,matrix_ex)
    print("dot product of camera intrinsic matrix and external matrix :")
    print(matrix)
    matrix_I = matrix.I
    print("inverse matrix:")
    print(matrix_I)
    # worldcoord = k_I*pixel*R_I -t*R_I
    worldcoord = matrix_I*pixel_T
    worldcoord = worldcoord/worldcoord[3]
    trans =np.mat([[0.39412314633],[0.22151704122],[0.150124589124],[1]])

    print("trans:")
    print(trans)

#    return worldcoord

if __name__ == "__main__":
   #rotate matrix calculated by get_rotateMatrix.py
    R = np.mat([[ -2.03673204e-04 , 9.99999804e-01   ,5.92653543e-04],
                [ -9.99999979e-01 ,  2.03673168e-04  ,-1.20707648e-07],
                [  0.00000000e+00 , -5.92653555e-04  ,-9.99999824e-01],])

    #camera_parameters gotten from rostopic echo KinectV2/rgb/camera_info
    K= np.mat(   [[589.3671569219632 , 0.0               , 320.5],
                  [0.0               , 589.3671569219632 , 240.5],
                  [0.0               , 0.0               , 1.0 ]])
    t = np.mat([[-0.443], [0.095], [-0.968]]) 
    print("t:") 
    print(t)
    matrix_ex = np.hstack((R,t))
    matrix_ex = np.vstack((matrix_ex,[0,0,0,1])) 
    print("matrix_ex:")
    print(matrix_ex)
    # pixel coordinate(u,v,1)
    pixel = np.mat([86, 274 ,1 ])
    worldcoord = pixelcoord2worldcoord(K,matrix_ex,pixel)
    print(worldcoord)
