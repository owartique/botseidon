//#include <stdio.h>
#include <stdlib.h>
#include "CtrlStruct_gr3.h"
#include "speed_controller.h"


void init_speed_controller(CtrlStruct* theCtrlStruct){
   	theCtrlStruct->theUserStruct = (struct UserStruct *) malloc(sizeof(struct UserStruct));
	theCtrlStruct->theUserStruct->Un = 24.0;
	theCtrlStruct->theUserStruct->In = 0.78;
	theCtrlStruct->theUserStruct->Nn = 6470.0;
	theCtrlStruct->theUserStruct->Ra = 7.1;
	theCtrlStruct->theUserStruct->R = 0.03;
	theCtrlStruct->theUserStruct->kphi = 0.0261;
	theCtrlStruct->theUserStruct->dt = 0.04;
	theCtrlStruct->theUserStruct->red = 14.0;
   	theCtrlStruct->theUserStruct->theMotor_l = (struct Motor *) malloc(sizeof(struct Motor));
    	theCtrlStruct->theUserStruct->theMotor_r = (struct Motor *) malloc(sizeof(struct Motor));

	theCtrlStruct->theUserStruct->theMotor_l->Ki = 0.1194;
	theCtrlStruct->theUserStruct->theMotor_l->Kp = 0.0391;
	theCtrlStruct->theUserStruct->theMotor_l->K = 1.0;
	theCtrlStruct->theUserStruct->theMotor_l->komega = 0.9998;
	theCtrlStruct->theUserStruct->theMotor_l->integral_error = 0.0;

	theCtrlStruct->theUserStruct->theMotor_r->Ki = 0.1194;
	theCtrlStruct->theUserStruct->theMotor_r->Kp = 0.0391;
	theCtrlStruct->theUserStruct->theMotor_r->K = 1.0;
	theCtrlStruct->theUserStruct->theMotor_r->komega = 0.9998;
	theCtrlStruct->theUserStruct->theMotor_r->integral_error = 0.0;

	return;
}

