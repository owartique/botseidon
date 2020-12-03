#include <cstdio>
#include <time.h>
#include <string.h>

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
const float DELTA = 500.0;    // [mm]
const float ALPHA = 15.0;     // [deg]
const int SPEED = 25;

float leftSpeed = 0.0;
float rightSpeed = 0.0;

bool isMoving = false;
bool obstacle;
u_result     op_result;
rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();
CAN *can = new CAN(CAN_BR);
SPI_DE0 *spi = new SPI_DE0(0,500000);



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
    Call this function when lidar is not used anymore
*/
void stopLidar(){
    lidar->stop();
    lidar->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
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
    return 0 if beacon is closer than DELTA
           1 if beacon is on the left
           2 if beacon is on the right
           3 if beacon is further than DELTA on the front
           -1 otherwise
*/
int whereIsBeacon(){
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
                    if(angle>ALPHA & angle<180.f){
                        if(dist<dist_min){
                            dist_min = dist;
                            result = 2;
                        }
                    }
                    else if(angle<(360.f-ALPHA) & angle>180.f){
                        if(dist<dist_min){
                            dist_min = dist;
                            result = 1;
                        }
                    }
                    else if(angle>(360.f-ALPHA) | angle<ALPHA & dist>DELTA){
                        if(dist<dist_min){
                            dist_min = dist;
                            result = 3;
                        }
                    }
                    else if(dist<DELTA){//} & angle<90.0 | angle>(180.f)){
                        return 0;
                    }
         	 }
        }
    }
    return result;
}

/*
    encoder
*/
void encoder(){
    const int LEFT_ENCODER_ADR  = 0;
    const int RIGHT_ENCODER_ADR = 1;
    const int PPR  = 1024;
    const int BIAS = 2147483647; 
    const float PI = 3.141592654;
    const float R  = 0.03;
    const float DT = 0.02; // temps entre chaque prise de donnée des encodeurs = 50MHz/1e6 = 0.02 secondes

    // De ce que j'ai compris, le premier argument est le channel (0 ou 1) car le raspberry a deux channel SPI
    // ici on va toujours utiliser la channel 0
    // Le deuxième argument est la donnée qu'on veut envoyer. C'est un int et la fonction writeSPI va le transformer en bytes donc 
    // pas besoin de s'emmerder avec la conversion. 
    // Ici on envoie la donnée LEFT_ENCODER_ADR = 1. Le DE0 Nano va recevoir le chiffre de 32bits 0000 0000 0000 0000 0000 0000 0000 0001
    /////spi->writeSPI(0,LEFT_ENCODER_ADR);
    // Grâce au case(DataFromPI) dans le module sv le DE0 assigne que le DataToPI est le nombre de tick de la roue gauche
    // On lit ensuite ce que le DE0 renvoie
    unsigned int leftEncoder  = spi->readSPIolivier(0);
    // même chose pour la roue droite
    /////spi->writeSPI(0,RIGHT_ENCODER_ADR);
    unsigned int rightEncoder  = spi->readSPIolivier(1);
    // calcule de la vitesse en rad/s en tenant compte du biais introduit dans le DE0
    //leftSpeed = 2*PI*(leftEncoder)/(0.02*PPR);
    //rightSpeed = 2*PI*(rightEncoder)/(0.02*PPR);
    leftSpeed = 2*PI*R*((leftEncoder-BIAS)/PPR);
    rightSpeed = 2*PI*R*((rightEncoder-BIAS)/PPR);
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
        printf("stop\n");
    }
    else if(i==0 & !isMoving){
        printf("stop\n");
    }
    else if(i==1 & isMoving){
        can->push_PropDC(0,SPEED);
        printf("turn left\n");
    }
    else if(i==1 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(0,SPEED);
        isMoving = true;
        printf("turn left\n");
    }
    else if(i==2 & isMoving){
        can->push_PropDC(SPEED,0);
        printf("turn right\n");
    }
    else if(i==2 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(SPEED,0);
        isMoving = true;
        printf("turn right\n");
    }
    else if(i==3 & isMoving){
        can->push_PropDC(SPEED,SPEED);
        printf("move straight\n");
    }
    else if(i==3 & !isMoving){
        can->ctrl_motor(1);
        can->push_PropDC(SPEED,SPEED);
        isMoving = true;
        printf("move straight\n");
    }
    else{
        can->ctrl_motor(0);
        isMoving = false;
        printf("Invalid, motors OFF for safety \n");
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
	can->ctrl_led(1);
	can->ctrl_motor(1);
    can->push_PropDC(20,20);

	signal(SIGINT, ctrlc);

    while (1){
        //move(whereIsBeacon());

        encoder();
        printf("%f \t %f \n",leftSpeed,rightSpeed);

        if (ctrl_c_pressed){
                break;
        }
    }


	can->ctrl_led(0);
	can->ctrl_motor(0);
    stopLidar();


	return 0;
}
