# Author : Alexandre - Pierlot
# Modified by Olivier Wartique

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
import time

from tcpcom import TCPServer

#Flags
matchStarted = False
compassStabilized = False

#Variables
south = 0
north = 0
IP_PORT = 22000
start = None

R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = 1.0
R_flip[2,2] = 1.0

MarkerLenght_bot=7 #size of the marker on the bot
MarkerLenght_beacon=7 #size of the marker on the beacon

def onStateChanged(state, msg):
    if state == "LISTENING":
        print("Server:-- Listening...")
    elif state == "CONNECTED":
        print ("Server:-- Connected to", msg)
    elif state == "MESSAGE":
        print ("Server:-- Message received:", msg)
        if (msg=="start"): 
            matchStarted = True
            start = time.time()

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


def loadCalibration():
    cal= open('./calibration.pckl', 'rb')
    if cal==None:
        println("Error: no calibration file detected")
        os.exit()
        
    CameraMatrix,distCoef,_,_= pickle.load(cal) #get the camera matrix and distortion coefficients of the camera
    #print(CameraMatrix)

def setupCamera():
    video= cv.VideoCapture(0) #setting the camera object
    video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    Dict=aruco.Dictionary_get(aruco.DICT_4X4_100) #loading the dictionary containing the aruco markerss
    param=aruco.DetectorParameters_create() #create new parameters to the detection metod
                                            #parameters can still be changed for optimization

    font=cv.FONT_HERSHEY_PLAIN #font to write on the image





# =========================

loadCalibration()
setupCamera()

server = TCPServer(IP_PORT, stateChanged=onStateChanged)

while True:
    if(matchStarted):
        if(time.time()-start>10 and time.time()-start<25):
            ret, frame= video.read() #reading the camera frame
            gray= cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #converting the frame into a gray frame
            
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=Dict, parameters=param) #,cameraMatrix=CameraMatrix, distCoeff=distCoef)
            
            if ids is not None:
                i=0
                for i in (0, len(ids)-1):
                    if(ids[i]==17):
                        
                        Rvec,Tvec =aruco.estimatePoseSingleMarkers(corners[i],MarkerLenght_beacon,CameraMatrix,distCoef) #give the estimated rotation and translation vectors
                        aruco.drawDetectedMarkers(frame,[corners[i]])
                        aruco.drawAxis(frame,CameraMatrix,distCoef,Rvec,Tvec,5)
                        T_vec_boussole = Tvec[0,0]
                        R_ct = np.matrix(cv.Rodrigues(Rvec)[0])
                        R_tc = R_ct.T
                        x_rot,y_rot,z_rot = rotationMatrixToEulerAngles(R_flip*R_tc)
                        
                        #print(x_rot,y_rot,z_rot)
                        
                        if(x_rot<-2 and z_rot>2):
                            #print("SOUTH")
                            south += 1
                        elif(x_rot<0):
                            #print("NORTH")
                            north += 1
                    i= i+1

            cv.imshow('frame',frame) #display the frame
        
            key = cv.waitKey(1) & 0xFF #press q to quit
            if key == ord('q'):
                video.release()
                cv.destroyAllWindows()
                break
        elif (time.time()-start>25):
            # match finished
            if(north>south):
                server.sendMessage("NORTH")
            else:
                server.sendMessage("SOUTH")
            matchStarted = False
            server.terminate()
            video.release()
            cv.destroyAllWindows()
            break



