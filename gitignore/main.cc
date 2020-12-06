#include <cstdio>
#include <time.h>
#include <string.h>
#include <cmath>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>

#include "IO/CtrlStruct_gr3.h"
#include "IO/speed_controller.h"

//#include "IO/ctrl_io.h"

#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"

#include "IO/COM/SPI/Specific/SPI_DE0.hh"

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
const float DELTA = 500.0;      // [mm]
const float ALPHA = 15.0;       // [deg]
const int DC = 25;              // dutycycle of the propulsion motors

float leftSpeedLinear = 0.0;    // linear speed of the left wheel [m/s]
float rightSpeedLinear = 0.0;   // linear speed of the right wheel [m/s]

float leftSpeedAngular = 0.0;   // angular speed of the left wheel [rad/s]
float rightSpeedAngular = 0.0;  // angular speed of the right wheel [rad/s]

bool isMoving = false;          // variable to keep track if the minibot is moving or not
bool obstacle;
u_result     op_result;
rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
CAN *can = new CAN(CAN_BR);             // pointer to CAN
SPI_DE0 *spi = new SPI_DE0(0,500000);   // pointer to SPI



    



// ==============================================================================================================================================

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

// ==============================================================================================================================================

/*
    Call this function when lidar is not used anymore
        * stop the motor
        * get rid of the driver
*/
void stopLidar(){
    lidar->stop();
    lidar->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
}

// ==============================================================================================================================================

/*
    Return true if at least one obstacle in the last rotation of the lidar
*/
bool refreshData(float ang, float delta){
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
                if(angle<=ang & (angle>=(360.0-ang) | dist<delta)){
                    return true;
                }
        	 }
	    }
    }
    return false;
}

// ==============================================================================================================================================

/*
    beacon detection
*/
void detect(){
    obstacle = refreshData(ALPHA, DELTA);
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

// ==============================================================================================================================================

/*
    return 0 if beacon is closer than DELTA
           1 if beacon is on the left
           2 if beacon is on the right
           3 if beacon is further than DELTA on the front
           -1 otherwise
*/
int whereIsBeacon(float ang, float delta){
    float dist_min = 10000;
    int result = -1;
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
                    if(angle>ang & angle<180.f){
                        if(dist<dist_min){
                            dist_min = dist;
                            result = 2;
                        }
                    }
                    else if(angle<(360.f-ang) & angle>180.f){
                        if(dist<dist_min){
                            dist_min = dist;
                            result = 1;
                        }
                    }
                    else if(angle>(360.f-ang) | angle<ALPHA & dist>delta){
                        if(dist<dist_min){
                            dist_min = dist;
                            result = 3;
                        }
                    }
                    else if(dist<delta){//} & angle<90.0 | angle>(180.f)){
                        return 0;
                    }
         	 }
        }
    }
    return result;
}

// ==============================================================================================================================================

/*
    encoder
*/
void encoder(){
    const int LEFT_ENCODER_ADR  = 0;
    const int RIGHT_ENCODER_ADR = 1;
    const int PPR_LEFT  = 2048;//4096;          
    const int PPR_RIGHT = 2048;
    const int BIAS = 4092;          
    //const float PI = 3.141592654; // using M_PI instead (from cmath library)
    const float R  = 0.03;          // wheel radius [m]
    const float DT = 0.02;          // refresh rate of 0.02 seconds or 50 Hz

    // receive data from DE0 via SPI
    int leftEncoder  = spi->readSPIolivier(LEFT_ENCODER_ADR);
    leftEncoder  = spi->readSPIolivier(LEFT_ENCODER_ADR);
    int rightEncoder = spi->readSPIolivier(RIGHT_ENCODER_ADR);
    rightEncoder = spi->readSPIolivier(RIGHT_ENCODER_ADR);

    //linear speed [m/s]
    leftSpeedLinear  = 2*M_PI*R*((leftEncoder-BIAS)/(0.02*PPR_LEFT*14));
    while(leftSpeedLinear<-2 | leftSpeedLinear > 2){
        leftEncoder  = spi->readSPIolivier(LEFT_ENCODER_ADR);
        leftEncoder  = spi->readSPIolivier(LEFT_ENCODER_ADR);
        leftSpeedLinear  = 2*M_PI*R*((leftEncoder-BIAS)/(0.02*PPR_LEFT*14));
    }
    rightSpeedLinear = 2*M_PI*R*((rightEncoder-BIAS)/(0.02*PPR_RIGHT*14));
    while(rightSpeedLinear<-2 | rightSpeedLinear > 2){
        rightEncoder  = spi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightEncoder  = spi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightSpeedLinear  = 2*M_PI*R*((rightEncoder-BIAS)/(0.02*PPR_RIGHT*14));
    }

    // angular speed [rad/s]
    leftSpeedAngular  = leftSpeedLinear/R;
    rightSpeedAngular = rightSpeedLinear/R;
}

// ==============================================================================================================================================

/*
    if i = 0 stop
       i = 1 turn left
       i = 2 turn right
       i = 3 move straight
    Print the movements on the console is verbose is true
*/
void move(int i, bool verbose){
    if(i==0 & isMoving){
        can->ctrl_motor(0);
        isMoving = false;
        if(verbose) printf("stop\n");
    }
    else if(i==0 & !isMoving){
        if(verbose) printf("stop\n");
    }
    else if(i==1 & isMoving){
        can->push_PropDC(0,DC);
        if(verbose) printf("turn left\n");
    }
    else if(i==1 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(0,DC);
        isMoving = true;
        if(verbose) printf("turn left\n");
    }
    else if(i==2 & isMoving){
        can->push_PropDC(DC,0);
        if(verbose) printf("turn right\n");
    }
    else if(i==2 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(DC,0);
        isMoving = true;
        if(verbose) printf("turn right\n");
    }
    else if(i==3 & isMoving){
        can->push_PropDC(DC,DC);
        if(verbose) printf("move straight\n");
    }
    else if(i==3 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(DC,DC);
        isMoving = true;
        if(verbose) printf("move straight\n");
    }
    else{
        can->ctrl_motor(0);
        isMoving = false;
        if(verbose) printf("Invalid, motors OFF\n");
    }
}

// ==============================================================================================================================================

/*
    Print the welcome message on console
*/
void welcome(){
    printf("########################################################################################\n");
    printf("\t Welcome to the Minibot project of the ELEME2002 class :)\n");
    printf("########################################################################################\n");
    printf("\t I'm Mister White Spirit, please take care of me !\n");
    printf("\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    printf("\t Try to respect the C-file interface when programming me because\n \t it will be the same in the robotic project (Q2) !\n");
}

// ==============================================================================================================================================

int main(int argc, char** argv){
    welcome();
    
    lidarConfiguration();
    can->configure();
    can->ctrl_led(1);
    //can->ctrl_motor(0);
    //can->push_PropDC(20,20);
    can->ctrl_motor(1);
    


    // Controller
    CtrlStruct *ctrl = (struct CtrlStruct*)malloc(sizeof(CtrlStruct));

    init_speed_controller(ctrl);
    
    double omega_ref[2];
    double *omega_ref_ptr = omega_ref;
    omega_ref_ptr[0] = 7.0; 
    omega_ref_ptr[1] = 7.0; 
    run_speed_controller(ctrl,omega_ref_ptr);
    

    //printf("%f et %f\n",ctrl->theUserStruct->theMotor_l->Ki,ctrl->theUserStruct->theMotor_r->komega);

    while (1){
        
        //move(whereIsBeacon(ALPHA,DELTA),false);
        encoder();
        ctrl->theCtrlIn->l_wheel_speed = leftSpeedAngular;
        ctrl->theCtrlIn->r_wheel_speed = rightSpeedAngular;

        run_speed_controller(ctrl,omega_ref_ptr);
        can->push_PropDC(ctrl->theCtrlOut->wheel_commands[L_ID],ctrl->theCtrlOut->wheel_commands[R_ID]);
        printf("%.4f \t %.4f\n",leftSpeedAngular,rightSpeedAngular);
        sleep(ctrl->theUserStruct->dt);
        if (ctrl_c_pressed){
                break;
        }
    }
	
    printf("hello\n");
    sleep(0.1);

    can->ctrl_led(0);
    can->ctrl_motor(0);
    stopLidar();
    return 0;
}
