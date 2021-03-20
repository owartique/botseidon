#Alexandre - Pierlot

#EPL - Master 1 Mechatronics - 2020-2021
#Aruco marker detection and position estimation via webcam

#DOCUMENTATION:
#aruco: https://docs.opencv.org/4.2.0/d9/d6a/group__aruco.html
#openCV: https://docs.opencv.org

#To be able to use this code, you sould install the following packages via pip:
#opencv-contrib-python, numpy

import cv2 as cv
import cv2.aruco as aruco
import os, pickle
import Functions_aruco
import numpy as np
import math
import paramiko
import time

ssh_client= paramiko.SSHClient() #create a client to connect via SSH
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) #allow to connect to an unknown server
ssh_client.connect("10.3.141.1", port=22, username="pi", password="raspberry") #connect to the server via SSH 


#Check if the matrix is a rotation matrix
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


R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = 1.0
R_flip[2,2] = 1.0

minibot_id=2
beacon_id=0
MarkerLenght_bot=10 #size of the marker on the bot
MarkerLenght_beacon=10 #size of the marker on the beacon

cal= open('./calibration.pckl', 'rb')
if cal==None:
    println("Error: no calibration file detected")
    os.exit()
    
CameraMatrix,distCoef,_,_= pickle.load(cal) #get the camera matrix and distortion coefficients of the camera

video= cv.VideoCapture(0) #setting the camera object
video.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
video.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

Dict=aruco.Dictionary_get(aruco.DICT_4X4_100) #loading the dictionary containing the aruco markerss
param=aruco.DetectorParameters_create() #create new parameters to the detection metod
                                        #parameters can still be changed for optimization

font=cv.FONT_HERSHEY_PLAIN #font to write on the image

while True:
    ret, frame= video.read() #reading the camera frame
    gray= cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #converting the frame into a gray frame
    
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=Dict, parameters=param,
                              cameraMatrix=CameraMatrix, distCoeff=distCoef)
    #ids= [[3],[2],[5]]
   
    if ids is not None:
        i=0
        for i in (0, len(ids)-1):
            if(ids[i]==0): 
                Beacon=1
                Rvec,Tvec,_ =aruco.estimatePoseSingleMarkers(corners[i], MarkerLenght_beacon, CameraMatrix, distCoef) #give the estimated rotation and translation vectors
            elif ids[i]==2:
                minibot=1
                Rvec,Tvec,_ =aruco.estimatePoseSingleMarkers(corners[i], MarkerLenght_bot, CameraMatrix, distCoef) #give the estimated rotation and translation vectors
            #corners =[array([[[545., 372.],[278., 446.],[220., 143.],[489., 108.]]], dtype=float32), array([[[...]]], dtype=float32)]
            aruco.drawDetectedMarkers(frame,[corners[i]])
            aruco.drawAxis(frame, CameraMatrix, distCoef, Rvec,Tvec, 5)
            if(ids[i]==beacon_id): #if the id of the marker is the beacon
                T_vec_Beacon= Tvec[0,0]
            elif(ids[i]==minibot_id):
                T_vec_minibot= Tvec [0,0]
                R_ct    = np.matrix(cv.Rodrigues(Rvec)[0])
                R_tc    = R_ct.T
                x_rot, y_rot, z_rot = rotationMatrixToEulerAngles(R_flip*R_tc)

            i= i+1
    if(ids is not None and Beacon and minibot):
        order= Functions_aruco.AngleFind(T_vec_minibot[0],T_vec_minibot[1], T_vec_Beacon[0], T_vec_Beacon[1], z_rot)
        Command = "python3 Desktop/MyPI-Nano/Minibot.py " + order[0]
        print(Command)
        ssh_client.exec_command(Command)
        time.sleep(0.05)
        Beacon=0
        minibot=0
    cv.imshow('frame',frame) #display the frame
    
    
    
    key = cv.waitKey(1) & 0xFF #press q to quit
    if key == ord('q'):
        video.release()
        cv.destroyAllWindows()
        break