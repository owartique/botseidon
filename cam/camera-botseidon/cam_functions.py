# Author : Olivier Wartique

# Useful functions for the camera module of Botseidon

import cv2 as cv
import cv2.aruco as aruco
import os
import pickle
import Functions_aruco
import numpy as np 
import math
import time

# =============================================================

#Check if the matrix is a rotation matrix
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# =============================================================

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

# =============================================================

# Load the calibration file 
# return the camera matrix and the distortion coeficients
def loadCalibration(filename):
	cal = open(filename,'rb')
	if cal==None:
		println("Error : calibration file not found")
		os.exit()

	CameraMatrix,distCoef,_,_ = pickle.load(cal)

	return CameraMatrix,distCoef

# =============================================================

# Setting up the camera object
# return camera object,dictionnary and parameters
def setupCamera(width,height):
	video = cv.VideoCapture(0)
	video.set(cv.CAP_PROP_FRAME_WIDTH,width)
	video.set(cv.CAP_PROP_FRAME_HEIGHT,height)
	Dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
	param = aruco.DetectorParameters_create()

	font=cv.FONT_HERSHEY_PLAIN #font to write on the image

	return video,Dict,param

# =============================================================

# Read the camera frame 
# return frame,corners,ids
def readCamera(camera,Dict,param,camMatrix,distCoef):
	ret,frame = camera.read()
	gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
	corners,ids,rejected = aruco.detectMarkers(image=gray, dictionary=Dict, parameters=param)
                              #cameraMatrix=camMatrix, distCoeff=distCoef)  
	return frame,corners,ids


# =============================================================

# Inspect the camera frame
# return true if theID is found 
# return x,y,z and draw on the frame
def inspectFrame(corners,ids,theID,markerLength,camMatrix,distCoef,frame):
	n = len(ids)-1
	for i in (0,n):
		if(ids[i]==theID):
			Rvec,Tvec = aruco.estimatePoseSingleMarkers(corners[i],markerLength,camMatrix,distCoef)
			aruco.drawDetectedMarkers(frame,[corners[i]])
			aruco.drawAxis(frame,camMatrix,distCoef,Rvec,Tvec,5)
			R_ct = np.matrix(cv.Rodrigues(Rvec)[0])
			R_tc = R_ct.T
			#x_rot,y_rot,z_rot = rotationMatrixToEulerAngles(np.eye(3,dtype=np.float32)*R_tc)
	cv.imshow('frame',frame)

# =============================================================

