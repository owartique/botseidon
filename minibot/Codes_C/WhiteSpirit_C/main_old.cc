#include <cstdio>
#include <time.h>

#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"

#include "/home/pi/slamware/rplidar_sdk-release-v1.12.0/sdk/sdk/include/rplidar.h"
#include "/home/pi/slamware/rplidar_sdk-release-v1.12.0/sdk/sdk/include/rplidar_driver.h"

#include <unistd.h>
#include <stdlib.h>


#define CAN_BR 125e3

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, char** argv)
{
	printf("##############################################################################################################\n");
    	printf("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)");
    	printf("##############################################################################################################\n");
    	printf("\t\t I'm Mister GrayCode, please take care of me !\n");
    	printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    	printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");


	CAN *can;
	can = new CAN(CAN_BR);
	can->configure();
	can->ctrl_led(0);
	can->ctrl_motor(1);
	can->push_PropDC(20,20);
	
	//delay2(5000000);
	//can->ctrl_motor(0);
	//printf("finito\n");
	
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
	
	
	signal(SIGINT, ctrlc);
	int Detection = 0;
	int pos1=0;
	float AngleDetect = 0.0;

	while (1) {
	        rplidar_response_measurement_node_hq_t nodes[8192];
		size_t nodeCount = sizeof(nodes)/sizeof(rplidar_response_measurement_node_hq_t);
		res = lidar->grabScanDataHq(nodes, nodeCount);
		if (IS_OK(res)) {	
	            lidar->ascendScanData(nodes, nodeCount);
		    if (Detection) {
			pos1=0;
			while(1){
				if(nodes[pos1].dist_mm_q2/4.0f < 300 & (((nodes[pos1].angle_z_q14 * 90.f / (1 << 14)) < AngleDetect +5.0) | ((nodes[pos1].angle_z_q14 * 90.f / (1 << 14)) > AngleDetect) )) {
	                		//printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
	                    		//	(nodes[pos1].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
	                    		//	(nodes[pos1].angle_z_q14 * 90.f / (1 << 14)), 
	                    		//	nodes[pos1].dist_mm_q2/4.0f,
	                    		//	nodes[pos1].quality);
					can->ctrl_led(1);
					can->ctrl_motor(0);
					pos1=pos1+1;

	            		}
				else if(nodes[pos1].dist_mm_q2/4.0f > 300 & (((nodes[pos1].angle_z_q14 * 90.f / (1 << 14)) < AngleDetect +5.0) | ((nodes[pos1].angle_z_q14 * 90.f / (1 << 14)) > AngleDetect)) ){
					pos1=pos1+1;
					printf("Bonswaaar \n") ;
					break;
				}

				else {
					pos1=pos1+1;
				}
				
			}
			Detection = 0;
		   }
		   else {
	            for (int pos = 0; pos < (int)nodeCount ; pos=pos+1) {
			if(nodes[pos].dist_mm_q2/4.0f < 300 & nodes[pos].quality != 0) {
	                	//printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
	                    	//	(nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
	                    	//	(nodes[pos].angle_z_q14 * 90.f / (1 << 14)), 
	                    	//	nodes[pos].dist_mm_q2/4.0f,
	                    	//	nodes[pos].quality);
				AngleDetect = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
				Detection = 1;
				break;
	            	}
		     }
		    can->ctrl_led(0);
	            can->ctrl_motor(1);
		   }
	
	        if (ctrl_c_pressed){ 
	            break;
	        }
	}
	}
	lidar->stop();
	lidar->stopMotor();
	can->ctrl_motor(0);

	rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
	lidar = NULL;
	return 0;
}
