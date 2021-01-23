

// Last modified Dec 13 2020 by Olivier

#include "customOli.hh"



// Address in SPI module of the DE0-Nano
const int LEFT_ENCODER_ADR  = 0;
const int RIGHT_ENCODER_ADR = 1;

// Useful constants
const int PPR_LEFT  = 2048*4;//2048;  // pulses per rotation (see Datasheet)          
const int PPR_RIGHT = 2048*4;
const int BIAS  = 4092;     	    // bias in the encoder (see DE0_Nano module)
const float PI = M_PI;     		// notation simplification
const float R  = 0.03;     		// wheel radius [m]
const float DT = 0.001;    		// refresh rate of 0.001 seconds (see DE0_Nano module) 
const float P = 2*M_PI*R; 			// perimeter of the wheel [m]
const float D = 0.22;				// distance [m] between the two wheels on the Minibot

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

   // printf("%f \t %i \t %i \t %f \t %i\n",M_PI,leftEncoder,BIAS,DT,PPR_LEFT );

    //linear speed [m/s]
    float leftAngularSpeed  = 2.0*M_PI*((leftEncoder-BIAS)/(DT*PPR_LEFT*14.0));

    //small cheat in order not to have unexpected values ;-)
    while(leftAngularSpeed<-3.0 | leftAngularSpeed > 3.0){
        leftEncoder = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
        leftEncoder = theSpi->readSPIolivier(LEFT_ENCODER_ADR);
        leftAngularSpeed = 2.0*M_PI*((leftEncoder-BIAS)/(DT*PPR_LEFT*14.0));
    }

    float rightAngularSpeed = 2.0*M_PI*((rightEncoder-BIAS)/(DT*PPR_RIGHT*14.0));
    while(rightAngularSpeed<-3.0 | rightAngularSpeed > 3.0){
        rightEncoder  = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightEncoder  = theSpi->readSPIolivier(RIGHT_ENCODER_ADR);
        rightAngularSpeed  = 2.0*M_PI*((rightEncoder-BIAS)/(DT*PPR_RIGHT*14.0));
    }

    // angular speed [rad/s]
    theCtrlStruct->theCtrlIn->l_wheel_speed = leftAngularSpeed;	
    theCtrlStruct->theCtrlIn->r_wheel_speed = rightAngularSpeed;

    // update the de number of ticks 
    *theLeftSum  = *theLeftSum+leftEncoder-BIAS;
    *theRightSum = *theRightSum+rightEncoder-BIAS;
}

// ==============================================================================================================================================

/*
	@inputs : 
		int radius       : the radius [mm] in which we are looking for the beacon
		int treshold 	 : the distance [mm] for which the minibot has to stop
		double* theAngle : pointer to the angle [deg] of the beacon
		bool* stop 	     : pointer to the variable stop

	@return: 
		bool : true if no beacon in radius, false otherwise
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

void stop(CtrlStruct* theCtrlStruct,double* theOmegaRef, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
	theOmegaRef[L_ID] = 0.0;
	theOmegaRef[R_ID] = 0.0;
    encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);  
    run_speed_controller(theCtrlStruct,theOmegaRef);
    theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
    usleep(50);
    while(theCtrlStruct->theCtrlIn->l_wheel_speed!=0.0 & theCtrlStruct->theCtrlIn->l_wheel_speed!=0.0){
        encoder(theCtrlStruct, theLeftSum, theRightSum, theSpi);  
        run_speed_controller(theCtrlStruct,theOmegaRef);
        theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
        sleep(0.04);
        //printf("%f   %f\n",ctrl->theCtrlIn->l_wheel_speed,ctrl->theCtrlIn->r_wheel_speed);
    }
    theCan->ctrl_motor(0); //ajout
}

// ==============================================================================================================================================

/*
	@inputs :
		double* theAngle 	      : pointer to the angle [deg] of the beacon
		bool* stop 		 		  : pointer to the variable stop
		double* theOmegaRef 	  : pointer to the reference speeds [rad/s]
		CtrlStruct* theCtrlStruct : pointer to the control structure of the controller
		CAN* theCan 			  : pointer to the can
		double* theLeftSum 		  : pointer to the left number of ticks since the begining of the program
		double* theRightSum       : pointer to the right number of ticks since the begining of the program
		SPI_DE0* theSpi           : pointer to the spi 

	@return : void
*/

void followTheBeacon(double* theAngle, bool* theObstacle, bool* theStop, double* theOmegaRef, CtrlStruct* theCtrlStruct, CAN* theCan, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi){
	bool noBeacon = whereIsBeacon(1500.0,300.0,theAngle,theObstacle);
	if(*theObstacle & !(*theStop)){
		stop(theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi);
		*theStop = true;
	}
	else if(noBeacon & *theStop){
		theOmegaRef[L_ID] = CRUISE_SPEED;
		theOmegaRef[R_ID] = CRUISE_SPEED;
		*theStop = false;
		theCan->ctrl_motor(1);
	}
	else if(!noBeacon & *theStop){
		rotate(theAngle, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
	}
	else if(!noBeacon & !(*theStop)){
		stop(theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi);
		rotate(theAngle, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
		*theStop = true;
	}
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
void rotate(double* theAngle, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){

	double ang;
	bool turnLeft;
	if(*theAngle>180.0){
		ang = (360.0-(*theAngle))*PI/180.0;
		turnLeft = true;
	}
	else{
		ang = *theAngle*(PI/180.0);
		turnLeft = false;
	}

	theOmegaRef[L_ID] = (turnLeft) ? -CRUISE_SPEED : CRUISE_SPEED;
	theOmegaRef[R_ID] = (turnLeft) ? CRUISE_SPEED : -CRUISE_SPEED;

	double lengthArc = ang*D/2; // longueur [m] de l'arc de cercle que les roues doivent parcourir
	double nTurns = lengthArc/P; // nombre de tours de roues
	double nTicksLeft = nTurns*PPR_LEFT*1.8;  // nombre de ticks correspondant au nombre de tours de roue
	double nTicksRight = nTurns*PPR_RIGHT*1.8; // nombre de ticks correspondant au nombre de tours de roue

	double startLeftSum = *theLeftSum;
	double startRightSum = *theRightSum;

	theCan->ctrl_motor(1);

	while((*theLeftSum<startLeftSum+nTicksLeft)&!turnLeft | (*theRightSum<startRightSum+nTicksRight)&turnLeft){
		encoder(theCtrlStruct,theLeftSum,theRightSum,theSpi);
		run_speed_controller(theCtrlStruct,theOmegaRef);
		theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
		sleep(0.04);
		//break;
	}

	stop(theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi); //ajout
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
void followMyBeacon(double* theAngle, bool* stop, double* theOmegaRef, CtrlStruct* theCtrlStruct, CAN* theCan, double* theLeftSum, double* theRightSum, SPI_DE0* theSpi){
	bool noBeacon = whereIsBeacon(1000.0,200.0,theAngle,stop);
	//printf("follow\n");
	if(*stop){
		printf("stop\n");
		theOmegaRef[L_ID] = 0.0;
		theOmegaRef[R_ID] = 0.0;
	}
	else if(noBeacon){
		printf("noBeacon\n");
		theOmegaRef[L_ID] = CRUISE_SPEED;
		theOmegaRef[R_ID] = CRUISE_SPEED;
		//run_speed_controller(theCtrlStruct, theOmegaRef);
		//theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
	}
	else if(!noBeacon){
		theOmegaRef[L_ID] = (*theAngle>180.0) ? -CRUISE_SPEED : CRUISE_SPEED;
		theOmegaRef[R_ID] = (*theAngle>180.0) ? CRUISE_SPEED : -CRUISE_SPEED;

	}
}
*/
// ==============================================================================================================================================
/*
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
        		if((angle>358.0 | angle<2.0) & dist<dist_ref){
        			*theCnt = *theCnt + 1;
        			stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
        			*theFirst = 1;
        			return;
        		}
        	}

		}
	}
}

// ==============================================================================================================================================

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
        		//printf("%i\n",cote );
        		if(cote){
        			if(angle>269.0 & angle<271.0 & dist<dist_ref){
        				*theCnt = *theCnt+1;
        				stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
        				*theFirst = 1;
        				return;
        			}
        		}
        		else if(cote==0){
        			//printf("%f    et   %f\n", dist_ref,dist );
        			if(angle>89.0 & angle<91.0 & dist<dist_ref){
        				printf("%s\n", "stop");
        				*theCnt = *theCnt+1;
        				stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
        				printf("%i\n", *theCnt);
        				*theFirst = 1;
        				return;
        			}        			
        		}
        	}
        }
    }
}


// ==============================================================================================================================================

void parcours(double* theOmegaRef, CtrlStruct* theCtrlStruct, int* theCnt, int* theFirst, int* theA, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){
	switch(*theCnt){
		case 0 : if(*theFirst){
					printf("%s\n", "je commence a reculer");
					theOmegaRef[L_ID] = -CRUISE_SPEED;
					theOmegaRef[R_ID] = -CRUISE_SPEED;
					*theFirst = 0;
				 }
				 else if(!*theFirst & *theA>10){
				 	while(!theCan->check_receive()){
				 	}
				 	*theCnt = *theCnt+1;
				 	stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 	*theFirst = 1;
					*theA = 0;
				 }
				 *theA = *theA+1;
				 break;
		case 1 : if(*theFirst){
					printf("%s\n", "je cherche si il y a rien a droite");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 lateral(1000.0,0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
		case 2 : printf("%s\n", "je tourne de 90 degres a droite");
				 rotate(90.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
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
					theOmegaRef[L_ID] = -CRUISE_SPEED;
					theOmegaRef[R_ID] = -CRUISE_SPEED;
					*theFirst = 0;
				 }
				 else if(!*theFirst & *theA>60){
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
		case 6 : if(*theFirst){
					printf("%s\n", "j'avance jusqu'a voir un objet a 2 m sur ma droite");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 }
				 lateral(1500.0,0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break; 
		case 7 : printf("%s\n", "je tourne de 90 degres a droite");
				 rotate(90.0, theOmegaRef, theCtrlStruct, theLeftSum, theRightSum, theCan, theSpi);
				 *theCnt = *theCnt+1;
				 stop(theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break; 
		case 8 : if(*theFirst){
					printf("%s\n", "j'avance jusqu'a etre a 20 cm devant la cible");
					theOmegaRef[L_ID] = CRUISE_SPEED;
					theOmegaRef[R_ID] = CRUISE_SPEED;
					*theFirst = 0;
				 } 
				 avancer(200.0,theCnt,theFirst,theCtrlStruct,theOmegaRef,theLeftSum,theRightSum,theCan,theSpi);
				 break;
	}
}	

*/

void follow(double* theAngle, bool* theObstacle, bool* rolling, CAN* theCan){
	bool noBeacon = whereIsBeacon(1500, 300, theAngle, theObstacle);
	if(*theObstacle & *rolling){
		//printf("%s\n", "stop");
		theCan->push_PropDC(0,0);
		theCan->ctrl_motor(0);
		*rolling = false;
	}
	else if(*theObstacle & !(*rolling)){
		//printf("%s\n", "stop");

	}
	else if((*theAngle>340.0 | *theAngle<20.0) & *rolling){
		//printf("%s\n", "devant");
		theCan->push_PropDC(20,20);
	}
	else if((*theAngle>340.0 | *theAngle<20.0) & !(*rolling)){
		//printf("%s\n", "devant");
		theCan->push_PropDC(20,20);
		theCan->ctrl_motor(1);
		*rolling = true;
	}
	else if((*theAngle>20.0 & *theAngle<90.0)& *rolling){
		//printf("%s\n", "droite");
		theCan->push_PropDC(20,0);
	}
	else if((*theAngle>20.0 & *theAngle<90.0)& !(*rolling)){
		//printf("%s\n", "droite");
		theCan->push_PropDC(20,0);
		theCan->ctrl_motor(1);
		*rolling = true;
	}
	else if((*theAngle>270.0 & *theAngle<340.0)& *rolling){
		//printf("%s\n", "gauche");
		theCan->push_PropDC(0,20);
	}
	else if((*theAngle>270.0 & *theAngle<340.0)& !(*rolling)){
		//printf("%s\n", "gauche");
		theCan->push_PropDC(0,20);
		theCan->ctrl_motor(1);
		*rolling = true;
	}
}


// ==============================================================================================================================================


void line(double dist, double* theOmegaRef, CtrlStruct* theCtrlStruct, double* theLeftSum, double* theRightSum, CAN* theCan, SPI_DE0* theSpi){

	double nTurns = dist/P; // nombre de tours de roues
	double nTicksLeft = nTurns*PPR_LEFT*1.8;  // nombre de ticks correspondant au nombre de tours de roue
	double nTicksRight = nTurns*PPR_RIGHT*1.8; // nombre de ticks correspondant au nombre de tours de roue

	double startLeftSum = *theLeftSum;
	double startRightSum = *theRightSum;

	theCan->ctrl_motor(1);

	while(*theLeftSum<startLeftSum+nTicksLeft | *theRightSum<startRightSum+nTicksRight){
		encoder(theCtrlStruct,theLeftSum,theRightSum,theSpi);
		run_speed_controller(theCtrlStruct,theOmegaRef);
		theCan->push_PropDC(theCtrlStruct->theCtrlOut->wheel_commands[L_ID],theCtrlStruct->theCtrlOut->wheel_commands[R_ID]);
		sleep(0.04);
		printf("%f \t %f\n", *theLeftSum, startLeftSum+nTicksLeft);
	}

	stop(theCtrlStruct, theOmegaRef, theLeftSum, theRightSum, theCan, theSpi); //ajout
}