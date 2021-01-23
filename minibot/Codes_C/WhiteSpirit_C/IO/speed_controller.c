#include <stdio.h>
#include <stdlib.h>
#include "CtrlStruct_gr3.h"
#include "speed_controller.h"


void init_speed_controller(CtrlStruct* theCtrlStruct){
   	theCtrlStruct->theUserStruct = (struct UserStruct *) malloc(sizeof(struct UserStruct));
	theCtrlStruct->theCtrlIn = (struct CtrlIn *) malloc(sizeof(struct CtrlIn));
	theCtrlStruct->theCtrlOut = (struct CtrlOut *) malloc(sizeof(struct CtrlOut));


	theCtrlStruct->theUserStruct->Un = 24.0;
	theCtrlStruct->theUserStruct->In = 0.7916;
	theCtrlStruct->theUserStruct->Nn = 6470.0;
	theCtrlStruct->theUserStruct->Ra = 7.1;
	theCtrlStruct->theUserStruct->R = 0.03;
	theCtrlStruct->theUserStruct->kphi = 0.0261;
	theCtrlStruct->theUserStruct->dt = 0.04;
	theCtrlStruct->theUserStruct->red = 23.0;
   	theCtrlStruct->theUserStruct->theMotor_l = (struct Motor *) malloc(sizeof(struct Motor));
    	theCtrlStruct->theUserStruct->theMotor_r = (struct Motor *) malloc(sizeof(struct Motor));

	theCtrlStruct->theUserStruct->theMotor_l->Ki = 0.2339;//0.1169;//1.169; //1.0;
	theCtrlStruct->theUserStruct->theMotor_l->Kp = 0.0522; //0.0261;
	theCtrlStruct->theUserStruct->theMotor_l->K = 1.0;
	theCtrlStruct->theUserStruct->theMotor_l->komega = 0.9998;
	theCtrlStruct->theUserStruct->theMotor_l->integral_error = 0.0;

	theCtrlStruct->theUserStruct->theMotor_r->Ki = 0.2339;//0.1169;//1.169;
	theCtrlStruct->theUserStruct->theMotor_r->Kp = 0.0522;//0.0261;//0.0622;
	theCtrlStruct->theUserStruct->theMotor_r->K = 1.0;
	theCtrlStruct->theUserStruct->theMotor_r->komega = 0.9998;
	theCtrlStruct->theUserStruct->theMotor_r->integral_error = 0.0;

	return;
}

void run_speed_controller(CtrlStruct* theCtrlStruct, double *omega_ref){
	
    double Ua_left_out = run_motor(theCtrlStruct,theCtrlStruct->theUserStruct->theMotor_l,omega_ref[L_ID],theCtrlStruct->theCtrlIn->l_wheel_speed);
    double Ua_right_out = run_motor(theCtrlStruct,theCtrlStruct->theUserStruct->theMotor_r,omega_ref[R_ID],theCtrlStruct->theCtrlIn->r_wheel_speed);

    theCtrlStruct->theCtrlOut->wheel_commands[L_ID] = Ua_left_out * 100.0 / theCtrlStruct->theUserStruct->Un;
    theCtrlStruct->theCtrlOut->wheel_commands[R_ID] = Ua_right_out * 100.0 / theCtrlStruct->theUserStruct->Un;

    //printf("Ua_left_out %f \n",Ua_left_out);
    //printf("Ua_right_out %f \n",Ua_right_out);

    return;
}

double run_motor(CtrlStruct* theCtrlStruct, Motor* theMotor, double omega_ref, double wheel_speed){
    	double motor_speed = wheel_speed / theMotor->komega * theCtrlStruct->theUserStruct->red;

	double delta_omega = omega_ref * theCtrlStruct->theUserStruct->red - motor_speed;

	double ua_p_ref = delta_omega * theMotor->Kp
	                    + theMotor->Ki * (theCtrlStruct->theUserStruct->dt * delta_omega + theMotor->integral_error);
	// Current limitation
	int current_sat = saturation(&ua_p_ref, theCtrlStruct->theUserStruct->Ra * theCtrlStruct->theUserStruct->In * 3);

	double ua_ref = ua_p_ref + theCtrlStruct->theUserStruct->kphi * motor_speed;

	// Voltage limitation
	int voltage_sat = saturation(&ua_ref, theCtrlStruct->theUserStruct->Un);

	if (!current_sat && !voltage_sat) {
	    theMotor->integral_error = theCtrlStruct->theUserStruct->dt * delta_omega + theMotor->integral_error;
	}

	double ua = (double) ua_ref / theMotor->K;

	return ua;
}

int saturation(double *x, double xsat){
    if (*x > xsat) {
	    *x = xsat;
	    printf("sature positive \n");
	    return 1;
	}
	else if (*x < -xsat ) {
	    *x = -xsat;
	    printf("sature negative \n");
	    return 1;
	}
	else {
	    return 0;
	}
}
