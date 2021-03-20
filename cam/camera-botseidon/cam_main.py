# Author : Olivier Wartique

# Main file for the camera module of Botseidon

import cv2 as cv
import cv2.aruco as aruco
import os
import pickle
import Functions_aruco
import numpy as np 
import math
import time
import cam_functions as cam 

# Variables
compassID = 17
markerLength = 10
calibFile = './calibration.pckl'
width =  640
height = 480
north = 0
south = 0

camMatrix,distCoef = cam.loadCalibration(calibFile)
video,Dict,param   = cam.setupCamera(width,height)

while True:

	frame,corners,ids = cam.readCamera(video,Dict,param,camMatrix,distCoef)
	#cam.inspectFrame = (corners,ids,compassID,markerLength,camMatrix,distCoef,frame)
	if(ids==None):
		print("no aruco markers detected")
	else:
		n = len(ids)-1
		for i in (0,n):
			if(ids[i]==compassID):
				Rvec,Tvec = aruco.estimatePoseSingleMarkers(corners[i],markerLength,camMatrix,distCoef)
				aruco.drawDetectedMarkers(frame,[corners[i]])
				aruco.drawAxis(frame,camMatrix,distCoef,Rvec,Tvec,5)
				R_ct = np.matrix(cv.Rodrigues(Rvec)[0])
				R_tc = R_ct.T
				x_rot,y_rot,z_rot = cam.rotationMatrixToEulerAngles(np.eye(3,dtype=np.float32)*R_tc)
				x_rot = np.rad2deg(x_rot)
				y_rot = np.rad2deg(y_rot)
				z_rot = np.rad2deg(z_rot)
				if(z_rot>-20 and z_rot<20):
					north += 1
				elif(z_rot>160):
					south += 1
				print('NORTH =',north,'SOUTH =',south)

	cv.imshow('Botseidon-Camera',frame)
	
	key = (cv.waitKey(1) and 0xFF)
	if(key==ord("q")):
		video.release()
		cv.destroyAllWindows()
		break


	