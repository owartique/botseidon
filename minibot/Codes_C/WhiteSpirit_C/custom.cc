// Last modified Dec 15 2020 by Augustin

#include "custom.hh"

// Address in SPI module of the DE0-Nano
const int LEFT_ENCODER_ADR  = 0;
const int RIGHT_ENCODER_ADR = 1;

// Useful constants
const int PPR_LEFT  = 2055*4;       // pulses per rotation (see Datasheet)
const int PPR_RIGHT = 2048*4;
const int BIAS  = 4092;     	    // bias in the encoder (see DE0_Nano module)
const float PI = M_PI;     		    // notation simplification
const float R  = 0.03;     		    // wheel radius [m]
const float DT = 0.001;    		    // refresh rate of 0.001 seconds (see DE0_Nano module)
const float P = 2*M_PI*R; 			// perimeter of the wheel [m]
const float D = 0.23;				// distance [m] between the two wheels on the Minibot

// Variables declaration
u_result op_result;
rp::standalone::rplidar::RPlidarDriver* lidar = rp::standalone::rplidar::RPlidarDriver::CreateDriver();

// ==============================================================================================================================================

/*
	update the angular speed (left and right)
	update the sum of ticks (left and right) since the begining of the program

	@inputs :
		CtrlStruct* theCtrlStruct : pointer to the controller
		double* theLeftSum : pointer to the left number of ticks since the begining of the program
		double* theRightSum : pointer to the right number of ticks since the begining of the program
		SPI_DE0* theSpie : pointer to the spi

	@return : void
*/
void encoder(CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi){

    // receive data from DE0 via SPI
    int leftEncoder  = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
    leftEncoder      = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
    int rightEncoder = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
    rightEncoder     = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);

    //linear speed [m/s]
    float leftAngularSpeed  = 2.0*M_PI*((leftEncoder-BIAS)/(DT*PPR_LEFT*23.0));

    //small cheat in order not to have unexpected values ;-)
    while(leftAngularSpeed<-3.0 | leftAngularSpeed > 3.0){
        leftEncoder = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
        leftEncoder = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
        leftAngularSpeed = 2.0*M_PI*((leftEncoder-BIAS)/(DT*PPR_LEFT*23.0));
    }

    float rightAngularSpeed = 2.0*M_PI*((rightEncoder-BIAS)/(DT*PPR_RIGHT*23.0));
    while(rightAngularSpeed<-3.0 | rightAngularSpeed > 3.0){
        rightEncoder  = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightEncoder  = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightAngularSpeed  = 2.0*M_PI*((rightEncoder-BIAS)/(DT*PPR_RIGHT*23.0));
    }

    // angular speed [rad/s]
    theCtrlStruct->theCtrlIn->l_wheel_speed = leftAngularSpeed;
    theCtrlStruct->theCtrlIn->r_wheel_speed = rightAngularSpeed;

    // update the de number of ticks
    *theLeftSum  = *theLeftSum+leftEncoder-BIAS;
    *theRightSum = *theRightSum+rightEncoder-BIAS;

}

void encoder2(CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi, vector<double> &left_speed, vector<double> &right_speed){

    // receive data from DE0 via SPI
    int leftEncoder  = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
    leftEncoder      = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
    int rightEncoder = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
    rightEncoder     = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);

    //linear speed [m/s]
    float leftAngularSpeed  = 2.0*M_PI*((leftEncoder-BIAS)/(DT*PPR_LEFT*23.0));

    //small cheat in order not to have unexpected values ;-)
    while(leftAngularSpeed<-3.0 | leftAngularSpeed > 3.0){
        leftEncoder = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
        leftEncoder = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
        leftAngularSpeed = 2.0*M_PI*((leftEncoder-BIAS)/(DT*PPR_LEFT*23.0));
    }

    float rightAngularSpeed = 2.0*M_PI*((rightEncoder-BIAS)/(DT*PPR_RIGHT*23.0));
    while(rightAngularSpeed<-3.0 | rightAngularSpeed > 3.0){
        rightEncoder  = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightEncoder  = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightAngularSpeed  = 2.0*M_PI*((rightEncoder-BIAS)/(DT*PPR_RIGHT*23.0));
    }

    // angular speed [rad/s]
    theCtrlStruct->theCtrlIn->l_wheel_speed = leftAngularSpeed;
    theCtrlStruct->theCtrlIn->r_wheel_speed = rightAngularSpeed;
    left_speed.push_back(leftAngularSpeed);
    right_speed.push_back(rightAngularSpeed);

    // update the de number of ticks
    *theLeftSum  = *theLeftSum+leftEncoder-BIAS;
    *theRightSum = *theRightSum+rightEncoder-BIAS;

}

// ==============================================================================================================================================

/*
	@inputs :
		double* theAngle 	      : pointer to the angle [deg] of the beacon
		double* theOmegaRef 	  : pointer to the reference speeds [rad/s]
		CtrlStruct* theCtrlStruct : pointer to the control structure of the controller
		double* theLeftSum		  : pointer to number of ticks left
		double* theRightSum       : pointer to number to ticks right
		CAN* theCan 			  : pointer to the can
		SPI_DE0* theSpi 		  : pointer to the spi

	@return : void
*/
void rotate(double theAngle, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
	
	double ang;
	bool turnLeft;
	if(theAngle>180.0){
		ang = (360.0-(theAngle))*PI/180.0;
		turnLeft = true;
	}
	else{
		ang = theAngle*(PI/180.0);
		turnLeft = false;
	}

	theOmegaRef[L_ID] = (turnLeft) ? -CRUISE_SPEED : CRUISE_SPEED;
	theOmegaRef[R_ID] = (turnLeft) ? CRUISE_SPEED : -CRUISE_SPEED;

	theCan->ctrl_motor(1);

	double lengthArc = ang*D/2; // longueur [m] de l'arc de cercle que les roues doivent parcourir
	double nTurns = lengthArc/P; // nombre de tours de roues
	double nTicksLeft = nTurns*PPR_LEFT*1.8;  // nombre de ticks correspondant au nombre de tours de roue
	double nTicksRight = nTurns*PPR_RIGHT*1.8; // nombre de ticks correspondant au nombre de tours de roue

	double startLeftSum = *theLeftSum;
	double startRightSum = *theRightSum;

        usleep(100);
	encoder(theCtrlStruct,theLeftSum,theRightSum,theSpi);
	run_speed_controller(theCtrlStruct,theOmegaRef);
	theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
	usleep(100);
	theCan->ctrl_motor(1);
	usleep(100);

	while((*theLeftSum<startLeftSum+nTicksLeft)&!turnLeft | (*theRightSum<startRightSum+nTicksRight)&turnLeft){
		encoder(theCtrlStruct,theLeftSum,theRightSum,theSpi);
		run_speed_controller(theCtrlStruct,theOmegaRef);
		theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
		sleep(0.05);
	}
}

// ==============================================================================================================================================

/*
    stop the motor
    free the driver

    @input : none
    @return : void
*/
void stopLidar(){
    lidar->stop();
    lidar->stopMotor();
    rp::standalone::rplidar::RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
}

// ==============================================================================================================================================

/*
    create a driver
    start the motor
    start the scan

    @input : none
    @return : void
*/
void lidarConfigure(){
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
    return true if a beacon is in the radius and gives the angle of the beacon, else return false
*/

bool whereIsBeacon(int radius, int threshold, double* theAngle, bool* theObstacle){
    double dist_min = (double) radius;
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
        lidar->ascendScanData(nodes, count);
        for (int pos = 0; pos < (int)count ; ++pos){
               double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
               double dist = nodes[pos].dist_mm_q2/4.0f;
            double quality = nodes[pos].quality;
            if(quality>0.0 & dist<dist_min){
                  dist_min = dist;
                  *theAngle = angle;
             }
        }
    }
    if(dist_min<threshold){
        *theObstacle = true;
    }
    else{
        *theObstacle = false;
    }
// if the minimum distance is the same as the radius then there is no beacon
    return (dist_min==radius);
}

// ==============================================================================================================================================
/*
    cnt = cnt + 1 if the robot see an object near from dist_ref front of him
*/

void avancer(double dist_ref, int* theCnt, int* theFirst,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
	rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
		lidar->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos){
       		double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
       		double dist = nodes[pos].dist_mm_q2/4.0f;
        	double quality = nodes[pos].quality;
        	if(quality>0.0){
        		if((angle>350.0 | angle<10.0) & dist<dist_ref){
        			*theCnt = *theCnt + 1;
        			stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
        			*theFirst = 1;
        			return;
        		}
        		encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
                run_speed_controller(theCtrlStruct,theOmegaRef);
                theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
                usleep(100);
        	}

		}
	}
}

// ==============================================================================================================================================
/*
    cnt = cnt + 1 if the robot see an object near from dist_ref in the specified side (0 -> right, 1 -> left)
*/
void lateral(double dist_ref, int cote, int* theCnt, int* theFirst,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
	rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
		lidar->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos){
       		double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
       		double dist = nodes[pos].dist_mm_q2/4.0f;
        	double quality = nodes[pos].quality;
        	if(quality>0.0){
        		if(cote){
        			if(angle>269.0 & angle<271.0 & dist<dist_ref){
        				*theCnt = *theCnt+1;
        				stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
        				*theFirst = 1;
        				return;
        			}
        		}
        		else if(cote==0){
        			if(angle>89.0 & angle<91.0 & dist<dist_ref){
        				printf("%s\n", "stop");
        				*theCnt = *theCnt+1;
        				stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
        				printf("%i\n", *theCnt);
        				*theFirst = 1;
        				return;
        			}
        		}
        		encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
                run_speed_controller(theCtrlStruct,theOmegaRef);
                theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
                usleep(100);
        	}
        }
    }
}


// ==============================================================================================================================================
/*
    implemented path that the robot follow for the demonstration of the minibot. The goal is to use a maximum of our capacities (functions) during the path.
*/
void parcours(double* theOmegaRef, CtrlStruct* theCtrlStruct, int* theCnt, int* theFirst, int* theA, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
	switch(*theCnt){
		case 0 : if(*theFirst){
					printf("%s\n", "je commence a reculer");
					theOmegaRef[L_ID] = -1.0;
					theOmegaRef[R_ID] = -1.0;
					*theFirst = 0;
				 }
				 else if(!*theFirst & *theA>500){
                    printf("je suis plus controle \n");
				 	while(!theCan->check_receive()){
				 	}
				 	*theCnt = *theCnt+1;
				 	stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 	*theFirst = 1;
					*theA = 0;
				 }
				 *theA = *theA+1;
				 break;
        case 1 : printf("j'avance de 30 cm \n");
                 line(0.3, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
                 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 2 : printf("%s\n", "je tourne de 90 degres a gauche");
				 rotate(270.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
                 encoder(theCtrlStruct,theLeftSum,theRightSum,theSpi);
                 run_speed_controller(theCtrlStruct,theOmegaRef);
                 theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
                 usleep(100);
				 break;
        case 3 : if(*theFirst){
					printf("%s\n", "j'avance jusqu'a etre a 30 cm de l'objet");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 avancer(300.0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 4 : printf("%s\n", "je fais un demi tour");
				 rotate(180.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
		case 5 : if(*theFirst){
					printf("%s\n", "je recule jusqu'au mur");
					theOmegaRef[L_ID] = -1.0;
					theOmegaRef[R_ID] = -1.0;
					*theFirst = 0;
				 }
				 else if(!*theFirst & *theA>500){
				 	while(!theCan->check_receive()){
				 	}
				 	*theCnt = *theCnt+1;
				 	stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 	*theFirst = 1;
					theOmegaRef[L_ID] = 0.0;
					theOmegaRef[R_ID] = 0.0;
					*theA = 0;
				 }
				 *theA = *theA+1;
				 break;
        case 6 : printf("j'avance de 60 cm \n");
                 line(0.6, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
                 *theCnt = *theCnt+1;
				 break;
        case 7 : printf("%s\n", "je tourne de 90 degres a gauche");
				 rotate(270.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 printf("j'avance de 30 cm \n");
                 line(0.3, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 break;
        case 8 : if(*theFirst){
					printf("%s\n", "j'avance jusqu'a voir un objet a 1 m sur ma gauche");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 lateral(1000.0,1,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 9 : printf("j'avance de 15 cm \n");
                 line(0.15, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
                 printf("%s\n", "je tourne de 90 degres a gauche");
				 rotate(270.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 10: printf("%s\n", "je fais un demi tour");
				 rotate(180.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 11: printf("j'avance de 70 cm \n");
                 line(0.7, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
                 *theCnt = *theCnt+1;
				 break;
        case 12: printf("%s\n", "je tourne de 90 degres a droite");
				 rotate(90.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 13: if(*theFirst){
					printf("%s\n", "j'avance jusqu'a voir un objet a 1.5 m sur ma gauche");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 lateral(1500.0,1,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 14: printf("j'avance de 15 cm \n");
                 line(0.1, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
                 printf("%s\n", "je tourne de 90 degres a gauche");
				 rotate(270.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 15: if(*theFirst){
					printf("%s\n", "j'avance jusqu'a etre a 30 cm de l'objet");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 avancer(300.0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 16: printf("%s\n", "je tourne de 135 degre a droite");
				 rotate(140.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
        case 17: if(*theFirst){
					printf("%s\n", "j'avance jusqu'a etre a 40 cm de l'objet");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 avancer(400.0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
	}
}

// ==============================================================================================================================================
/*
   stop the robot between each realized action
*/
void stop(CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
    /*theOmegaRef[L_ID] = 0.0;
    theOmegaRef[R_ID] = 0.0;
    encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
    run_speed_controller(theCtrlStruct,theOmegaRef);
    theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
    usleep(100);
    while(theCtrlStruct->theCtrlIn->l_wheel_speed!=0.0 & theCtrlStruct->theCtrlIn->l_wheel_speed!=0.0){
        encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
        run_speed_controller(theCtrlStruct,theOmegaRef);
        theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
        usleep(100);
    }
    theCan->ctrl_motor(0);
    sleep(1);
    theCan->push_PropDC(0,0);
    usleep(100);
    */
    theCan->ctrl_motor(0);
    theCtrlStruct->theUserStruct->theMotor_r->integral_error = 0.0;
    theCtrlStruct->theUserStruct->theMotor_l->integral_error = 0.0;
   
}

// ==============================================================================================================================================
/*
   the robot moves staight ahead "dist" meters
*/

void line(double dist, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){

	double nTurns = dist/P; // nombre de tours de roues
	double nTicksLeft = nTurns*PPR_LEFT*1.8;  // nombre de ticks correspondant au nombre de tours de roue
	double nTicksRight = nTurns*PPR_RIGHT*1.8; // nombre de ticks correspondant au nombre de tours de roue

	double startLeftSum = *theLeftSum;
	double startRightSum = *theRightSum;

	theOmegaRef[L_ID] = CRUISE_SPEED;
        theOmegaRef[R_ID] = CRUISE_SPEED;

	while(*theLeftSum<startLeftSum+nTicksLeft | *theRightSum<startRightSum+nTicksRight){
		encoder(theCtrlStruct,theLeftSum,theRightSum,theSpi);
		run_speed_controller(theCtrlStruct,theOmegaRef);
		theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
		usleep(100);
	}

	stop(theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi);
}

void line2(double dist, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi,vector<double> &left_speed, vector<double> &right_speed){

	double nTurns = dist/P; // nombre de tours de roues
	double nTicksLeft = nTurns*PPR_LEFT*1.8;  // nombre de ticks correspondant au nombre de tours de roue
	double nTicksRight = nTurns*PPR_RIGHT*1.8; // nombre de ticks correspondant au nombre de tours de roue

	double startLeftSum = *theLeftSum;
	double startRightSum = *theRightSum;

	theOmegaRef[L_ID] = CRUISE_SPEED;
    	theOmegaRef[R_ID] = CRUISE_SPEED;

	while(*theLeftSum<startLeftSum+nTicksLeft | *theRightSum<startRightSum+nTicksRight){
		encoder2(theCtrlStruct,theLeftSum,theRightSum,theSpi,left_speed, right_speed);
		run_speed_controller(theCtrlStruct,theOmegaRef);
		theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
		usleep(100);
	}

	//theCan->ctrl_motor(0);
	stop(theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi);
}

// ==============================================================================================================================================
/*
   idem lateral but "bullshit" replace the counter system
*/

void lateral2(double dist_ref, int cote,CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi, int* theBullshit){
	rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
		lidar->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos){
       		double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
       		double dist = nodes[pos].dist_mm_q2/4.0f;
        	double quality = nodes[pos].quality;
        	if(quality>0.0){
        		if(cote){
        			if(angle>269.0 & angle<271.0 & dist>dist_ref){
                        *theBullshit = 1;
        				return;
        			}
        		}
        		else if(cote==0){
        			if(angle>89.0 & angle<91.0 & dist>dist_ref){
                        *theBullshit = 1;
        				return;
        			}
        		}
        	}
        }
    }
}

// ==============================================================================================================================================
/*
   stop the robot between each realized action for the avoiding path
*/

void stopAvoid(CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
    theOmegaRef[L_ID] = 0.0;
    theOmegaRef[R_ID] = 0.0;
    encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
    run_speed_controller(theCtrlStruct,theOmegaRef);
    theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
    usleep(100);
    while(theCtrlStruct->theCtrlIn->l_wheel_speed!=0.0 & theCtrlStruct->theCtrlIn->l_wheel_speed!=0.0){
        encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
        run_speed_controller(theCtrlStruct,theOmegaRef);
        theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
        usleep(100);
    }
    theCan->ctrl_motor(0);
    sleep(0.5);
    theCan->push_PropDC(0,0);
    usleep(100);
    theCan->ctrl_motor(1);
    theCtrlStruct->theUserStruct->theMotor_r->integral_error = 0.0;
    theCtrlStruct->theUserStruct->theMotor_l->integral_error = 0.0;

    // ajoute mardi soir

}

// ==============================================================================================================================================
/*
   capture le data d'un tour de lidar
*/
int i = 1;
void captureData(vector<double> &rayon, vector<double> &alpha){
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t   count = _countof(nodes);
    op_result = lidar->grabScanDataHq(nodes, count);
    if (IS_OK(op_result)) {
		lidar->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos){
       		double angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
       		double dist = nodes[pos].dist_mm_q2/4.0f;
        	double quality = nodes[pos].quality;
        	if(quality>0.0){
        		rayon.push_back(dist);
        		alpha.push_back(angle);
        	}
        	if(angle>358.f){
                vector<pair<string , vector<double> > > data = {{"Distance", rayon}, {"Angle", alpha}};
                write_csv("log/lidar"+to_string(i)+".csv", data);
                rayon.clear();
                alpha.clear();
                i++;
                return;
        	}
        }
    }
}



// ==============================================================================================================================================
/*
   small path to show the avoidance of an obstacle
*/

void avoid(double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi,vector<double> &rayon, vector<double> &alpha){
    //avancer(200.0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
    captureData(rayon, alpha);

    printf("%s\n", "je tourne de 90 deg a droite");
    rotate(90.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);

    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);

    double startLeft = theSpi->readSPIolivier(2);
    double startRight = theSpi->readSPIolivier(3);

    theOmegaRef[L_ID] = CRUISE_SPEED;
    theOmegaRef[R_ID] = CRUISE_SPEED;

    int bullshit = 0;
    int* bullshit_ptr = &bullshit;
    while(*bullshit_ptr==0){
        lateral2(500.0, 1, theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi, bullshit_ptr);
        encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
        run_speed_controller(theCtrlStruct,theOmegaRef);
        theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
        usleep(100);
        printf("%s\n", "je suis bloque a gauche bro");
    }
    *bullshit_ptr=0;
    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);

    printf("%s\n", "j'avance de 30 cm");
    line(0.3, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);

    double cntLeft = theSpi->readSPIolivier(2) - startLeft;
    double cntRight = theSpi->readSPIolivier(3) - startRight;

    printf("%s\n", "je tourne de 90 deg a gauche");
    rotate(270.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);

    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
    captureData(rayon, alpha);

    printf("%s\n", "j'avance de 35 cm");
    line(0.35, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);

    theOmegaRef[L_ID] = CRUISE_SPEED;
    theOmegaRef[R_ID] = CRUISE_SPEED;

    while(*bullshit_ptr==0){
        lateral2(500.0, 1, theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi, bullshit_ptr);
        encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);
        run_speed_controller(theCtrlStruct,theOmegaRef);
        theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
        usleep(100);
        printf("%s\n", "je suis bloque a gauche bro");
    }

    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);

    printf("%s\n", "j'avance de 30 cm");
    line(0.3, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);

    captureData(rayon, alpha);

    printf("%s\n", "je tourne de 90 deg a gauche");
    rotate(270.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);

    double distLeft = P*cntLeft/(PPR_LEFT*1.8);         //pas logique, normalement : P*cnt/(nbr tick par tour de roue = 2048*4*23)
    double distRight = P*cntRight/(PPR_RIGHT*1.8);
    double dist = 0.5*(distLeft+distRight)/4.0;         //le 1.8, 0.5 et 4 donne 16 donc quasi le rapport de 23, d'où sort le 0.5? cntLeft vaut 0 en plus...

    printf("%s\n", "j'avance de ditst");
    line(dist, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);

    printf("%s\n", "je tourne de 90 deg puis j'ai fini");
    rotate(90.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);

    captureData(rayon, alpha);

    line(0.1, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);

}

// ==============================================================================================================================================
/*
   the function to follow the beacon (nearest object) without controller
*/

void follow(double* theAngle, bool* theObstacle, bool* rolling, CAN* theCan, string &msg){
    bool noBeacon = whereIsBeacon(1500, 300, theAngle, theObstacle);
    if(*theObstacle & *rolling){
        msg = "Demo 3 : stop\n";
        //printf("%s\n", "stop");
        theCan->push_PropDC(0,0);
        theCan->ctrl_motor(0);
        *rolling = false;
    }
    else if(*theObstacle & !(*rolling)){
        //printf("%s\n", "stop");
        msg = "Demo 3 : stop\n";
    }
    else if((*theAngle>350.0 | *theAngle<10.0) & *rolling){
        //printf("%s\n", "devant");
        msg = "Demo 3 : devant\n";
        theCan->push_PropDC(30,30);
    }
    else if((*theAngle>350.0 | *theAngle<10.0) & !(*rolling)){
        //printf("%s\n", "devant");
        msg = "Demo 3 : devant\n";
        theCan->push_PropDC(30,30);
        theCan->ctrl_motor(1);
        *rolling = true;
    }
    else if((*theAngle>10.0 & *theAngle<90.0)& *rolling){
        //printf("%s\n", "droite");
        msg = "Demo 3 : à droite\n";
        theCan->push_PropDC(30,0);
    }
    else if((*theAngle>10.0 & *theAngle<90.0)& !(*rolling)){
        //printf("%s\n", "droite");
        msg = "Demo 3 : à droite\n";
        theCan->push_PropDC(30,0);
        theCan->ctrl_motor(1);
        *rolling = true;
    }
    else if((*theAngle>270.0 & *theAngle<350.0)& *rolling){
        //printf("%s\n", "gauche");
        msg = "Demo 3 : à gauche\n";
        theCan->push_PropDC(0,30);
    }
    else if((*theAngle>270.0 & *theAngle<350.0)& !(*rolling)){
        //printf("%s\n", "gauche");
        msg = "Demo 3 : à gauche\n";
        theCan->push_PropDC(0,30);
        theCan->ctrl_motor(1);
        *rolling = true;
    }
}

// ==============================================================================================================================================
/*
   DEMO 1 : the robot moves staight ahead 3 meters
*/

void demo1(double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi, vector<double> &left_speed, vector<double> &right_speed){
    line2(3.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi, left_speed, right_speed);
}

// ==============================================================================================================================================
/*
   DEMO 2 : the robot rotate to 90 deg and after 180 deg
*/

void demo2(double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
    rotate(90.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
    rotate(181.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
    stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
}

// ==============================================================================================================================================
/*
   Prend un vecteur et l'ecrit dans un ficher .csv
*/

void write_csv(string filename, vector<pair<string, vector<double> > > dataset){
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    // Create an output filestream object
    ofstream myFile(filename);

    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}
