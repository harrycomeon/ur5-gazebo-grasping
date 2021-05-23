#!/usr/bin/env python
import numpy as np
import scipy.linalg as linalg
import math

ushort d = dep
def pixelcoord2worldcoord(K,R,t,pixel):

    pixel_T = pixel.transpose()
    print("pixel_T:")
    print(pixel_T)

    R_I = R.I
    print("R_I:")
    print(R_I)   

    K_I=K.I
    print("K_I:")
    print(K_I) 

    worldcoord = R_I*(K_I*(0.2261)*pixel_T-t)
    return worldcoord

if __name__ == "__main__":
#rotate matrix calculated by get_rotateMatrix.py
    R = np.mat(   [[  0 ,  0  , 1],
                  [   0 ,  1  , 0],
                  [  -1 ,  0  , 0]])
    #camera_parameters gotten from rostopic echo KinectV2/rgb/camera_info
    K= np.mat(   [[589.3671569219632 , 0.0               , 320.5],
                  [0.0               , 589.3671569219632 , 240.5],
                  [0.0               , 0.0               , 1.0  ]])
    t = np.mat([[400],[0],[1000]]) 
    print("t:") 
    print(t)
    pixel = np.mat([249.064648118 76.8870703764,1])
    worldcoord = pixelcoord2worldcoord(K,R,t,pixel)
    print(worldcoord)
