#include <cstdio>
#include <time.h>

#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"

#include "/home/pi/slamware/rplidar_sdk-release-v1.12.0/sdk/sdk/include/rplidar.h"
#include "/home/pi/slamware/rplidar_sdk-release-v1.12.0/sdk/sdk/include/rplidar_driver.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


#include <unistd.h>
#include <stdlib.h>


#define CAN_BR 125e3




/* Variables declaration */
const float DELTA = 300.0;    // [mm]
const float ALPHA = 10.0;     // [deg]
bool isMoving = false;
bool obstacle;
u_result     op_result;
rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
CAN *can = new CAN(CAN_BR);


/*
    Configuration of the lidar :
        * creates a driver
        * starts the motor
        * starts the scan
*/
void lidarConfiguration(){
    	u_result res = lidar->connect("/dev/ttyUSB0", 115200);
		if (IS_OK(res)){
    			printf("Success \n");
		}
		else{
		    fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
		}
	std::vector<rp::standalone::rplidar::RplidarScanMode> scanModes;
	lidar->getAllSupportedScanModes(scanModes);

	rp::standalone::rplidar::RplidarScanMode scanMode;
	lidar->startMotor();
	lidar->startScan(0,1);
}

/*
    Va chercher les donnees du dernier tour et renvoie true si il y a au moins un obstacle
*/
bool refreshData(){
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
        lidar->ascendScanData(nodes, count);
	    for (int pos = 0; pos < (int)count ; ++pos){
       		 float angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
       		 float dist = nodes[pos].dist_mm_q2/4.0f;
        	 float quality = nodes[pos].quality;
         	 if(quality>0.0){
                if(angle<=ALPHA & (angle>=(360.0-ALPHA) | dist<DELTA)){
                    return true;
                }
        	 }
	    }
    }
    return false;
}

/*
    beacon detection
*/
void detect(){
    obstacle = refreshData();
    if(isMoving & obstacle){ // si obstacle et mouvement alors stop
        can->ctrl_led(1);
        can->ctrl_motor(0);
        printf("stop\n");
        isMoving = false;
    }
    else if(!isMoving & !obstacle){ // si pas d'obstacle et à l'arret alors mouvement
        can->ctrl_led(0);
        can->ctrl_motor(1);
        printf("moving\n");
        isMoving = true;
    }
}

/*
    return 0 if beacon is closer than 300 mm
           1 if beacon is between 300 and 800 mm on the left
           2 if beacon is between 300 and 800 mm on the right
           3 if beacon is between 300 and 800 mm on the front
           -1 otherwise
*/
int whereIsBeacon(){
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
        lidar->ascendScanData(nodes, count);
	    for (int pos = 0; pos < (int)count ; ++pos){
       		 float angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
       		 float dist = nodes[pos].dist_mm_q2/4.0f;
        	 float quality = nodes[pos].quality;
         	 if(quality>0.0){
                    if(angle>20.f & angle<90.f & dist>DELTA & dist<800.f){
                        return 2;
                    }
                    else if(angle<340.f & angle>270.f & dist>DELTA & dist<800.f){
                        return 1;
                    }
                    else if(angle>340.f | angle<20.f & dist>DELTA & dist<800.f){
                        return 3;
                    }
                    else if(dist<DELTA){
                        return 0;
                    }
            }
	    }
    }
    return -1;
}

/*
    if i = 0 stop
       i = 1 turn left
       i = 2 turn right
       i = 3 move straight
*/
void move(int i){
    if(i==0 & isMoving){
        can->ctrl_motor(0);
        isMoving = false;
    }
    else if(i==1 & isMoving){
        can->push_PropDC(0,20);
    }
    else if(i==1 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(0,20);
        isMoving = true;
    }
    else if(i==2 & isMoving){
        can->push_PropDC(20,0);
    }
    else if(i==2 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(20,0);
        isMoving = true;
    }
    else if(i==3 & isMoving){
        can->push_PropDC(20,20);
    }
    else if(i==3 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(20,20);
        isMoving = true;
    }
    else if(i>3){
        can->ctrl_motor(0);
        isMoving = false;
        printf("Invalid entry, motor are turned of for safety\n");
    }
}


/*
    Print the welcome message on console
*/
void welcome(){
    printf("##############################################################\n");
    printf("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)");
    printf("##############################################################\n");
    printf("\t\t I'm Mister GrayCode, please take care of me !\n");
    printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");
}


int main(int argc, char** argv){
    welcome();
    lidarConfiguration();

	can->configure();
	can->ctrl_led(0);
	can->push_PropDC(20,20);
	can->ctrl_motor(0);

	signal(SIGINT, ctrlc);

    while (1){
        move(whereIsBeacon());
        if (ctrl_c_pressed){
                break;
        }
    }
	lidar->stop();
	lidar->stopMotor();
	can->ctrl_led(0);
	can->ctrl_motor(0);

	rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
	lidar = NULL;
	return 0;
}
