
// Last modified Dec 13 2020 by Olivier


#ifndef CUSTOM_H_OLI
#define CUSTOM_H_OLI

#include <cstdio>
#include <time.h>
#include <string.h>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "IO/CtrlStruct_gr3.h"
#include "IO/speed_controller.h"
#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"
#include "/home/pi/slamware/rplidar_sdk-release-v1.12.0/sdk/sdk/include/rplidar.h"
#include "/home/pi/slamware/rplidar_sdk-release-v1.12.0/sdk/sdk/include/rplidar_driver.h"

#define CAN_BR 125e3
#define CRUISE_SPEED 1.0

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


void encoder(CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi);  
bool whereIsBeacon(int radius, int threshold, double* theAngle, bool* theObstacle);
void stop(CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void followTheBeacon(double* theAngle, bool* theObstacle, bool* stop, double* theOmegaRef, CtrlStruct* theCtrlStruct, CAN* theCan, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi);
void rotate(double* theAngle, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void stopLidar();
void lidarConfigure();
//void followMyBeacon(double* theAngle, bool* stop, double* theOmegaRef, CtrlStruct* theCtrlStruct, CAN* theCan, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi);
//void avancer(double dist_ref, int* theCnt, int* theFirst,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
//void lateral(double dist_ref, int cote, int* theCnt, int* theFirst,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
//void parcours(double* theOmegaRef, CtrlStruct* theCtrlStruct, int* theCnt, int* theFirst, int* theA, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void follow(double* theAngle, bool* theObstacle, bool* rolling, CAN* theCan);
void line(double dist, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);


#endif