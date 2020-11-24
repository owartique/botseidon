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
const int DELTA   = 300;    // [mm]
const float ALPHA = 10;     // [deg]
bool isMoving;
u_result     op_result;


/*
    Configuration of the lidar :
        * creates a driver
        * starts the motor
        * starts the scan
*/
void lidarConfiguration(){
	rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();

    	u_result res = lidar->connect("/dev/ttyUSB0", 115200);

		if (IS_OK(res))
		{
    			printf("Success \n");

		}
		else
		{
		    fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
		}
	//rp::standalone::rplidar::RplidarScanMode scanModes;
	//lidar->rp::standalone::rplidar::RPlidarDriver::getAllSupportedScanModes(scanModes, 3);

	std::vector<rp::standalone::rplidar::RplidarScanMode> scanModes;
	lidar->getAllSupportedScanModes(scanModes);

	rp::standalone::rplidar::RplidarScanMode scanMode;
	//lidar->startScan(false, true, 0, &scanMode);
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
    }
    for (int pos = 0; pos < (int)count ; ++pos) {
        float angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
        int dist = nodes[pos].dist_mm_q2/4.0f;
        int quality = nodes[pos].dist_mm_q2/4.0f;

        if(quality>0){
            if(angle<=ALPHA & angle>=(360-ALPHA) & dist<DELTA){
                return true;
            }
        }
    }
    return false;
}

/*
    Print the welcome message on console
*/

void welcome(){
    printf("##############################################################################################################\n");
    printf("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)");
    printf("##############################################################################################################\n");
    printf("\t\t I'm Mister GrayCode, please take care of me !\n");
    printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");
}


int main(int argc, char** argv)
{

    welcome();
    lidarConfiguration();

	CAN *can;
	can = new CAN(CAN_BR);
	can->configure();
	can->ctrl_led(0);
	can->ctrl_motor(1);
	can->push_PropDC(20,20);

	signal(SIGINT, ctrlc);
	int Detection = 0;
	int pos1=0;
	float AngleDetect = 0.0;

    while (1){
        obstacle = refreshData();
        if(isMoving & obstacle){ // si obstacle et mouvement alors stop
            can->ctrl_motor(0);
            isMoving = false;
        }
        else if(!isMoving & !obstacle){ // si pas d'obstacle et à l'arret alors mouvement
            can->ctrl_motor(1);
            isMoving = true;
        }
        if (ctrl_c_pressed){
            break;
        }
    }
	lidar->stop();
	lidar->stopMotor();
	can->ctrl_motor(0);

	rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
	lidar = NULL;
	return 0;
}
