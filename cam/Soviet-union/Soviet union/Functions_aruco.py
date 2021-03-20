import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import os, pickle, math
import time

#checks the angles and tell the movement the robot should do
def AngleFind(x_minibot, y_minibot, x_beacon, y_beacon, angle_minibot):
    MtoB=[(x_beacon-x_minibot), (y_beacon-y_minibot)]
    angle_MtoB= np.arctan2(MtoB[1], MtoB[0])
    lenght_MtoB= math.sqrt(MtoB[0]**2 + MtoB[1]**2)

    angle_diff= math.degrees(angle_minibot-angle_MtoB)
    if(angle_diff > 10):
        #print("Turn clock")
        #print(angle_diff)
        return ["L", angle_diff]
    elif(angle_diff < -10):
        #print("Turn counterclock")
        #print(angle_diff)
        return ["R", angle_diff]
    elif lenght_MtoB < 1:
        return ["S", lenght_MtoB]
    else: #if forward
        print(lenght_MtoB)
        return ["F",lenght_MtoB]
