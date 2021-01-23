
// Last modified Dec 11 2020 by Olivier


#ifndef CUSTOM_H
#define CUSTOM_H

#include <cstdio>
#include <time.h>
#include <string>
using namespace std;
#include <vector>
#include <fstream>
#include <utility>
#include <sstream>

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
void encoder2(CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi, vector<double> &left_speed, vector<double> &right_speed);
void rotate(double theAngle, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void stopLidar();
void lidarConfigure();
bool whereIsBeacon(int radius, int threshold, double* theAngle, bool* theObstacle);
void avancer(double dist_ref, int* theCnt, int* theFirst,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void lateral(double dist_ref, int cote, int* theCnt, int* theFirst,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void parcours(double* theOmegaRef, CtrlStruct* theCtrlStruct, int* theCnt, int* theFirst, int* theA, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void stop(CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void line(double dist, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void line2(double dist, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi,vector<double> &left_speed, vector<double> &right_speed);
void lateral2(double dist_ref, int cote,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi, int* theBullshit);
void captureData(vector<double> &rayon, vector<double> &alpha);
void avoid(double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi,vector<double> &rayon, vector<double> &alpha);
void follow(double* theAngle, bool* theObstacle, bool* rolling, CAN* theCan, string &msg);
void demo1(double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi,vector<double> &left_speed, vector<double> &right_speed);
void demo2(double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi);
void write_csv(string filename, vector<pair<string, vector<double> > > dataset);

#endif
