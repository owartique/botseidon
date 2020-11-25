#include "controller.hh"
#include <cstdio>

#include <stdio.h>
#include <stdlib.h>
		
void init_speed_controller(CtrlStruct* theCtrlStruct){
	theCtrlStruct->theMotor->Un = 24.0;
	theCtrlStruct->theMotor->In = 0.72;								//c'est le courant max et pas nominal
	theCtrlStruct->theMotor->Nn = 7000;								//c'est la vitesse max et pas nominal
	theCtrlStruct->theMotor->Ra = 7.1;
	theCtrlStruct->theMotor->R = 0.03;
	theCtrlStruct->theMotor->kphi = 0.0261;
	theCtrlStruct->theMotor->dt = 0.04;								//jsp quoi choisir
	theCtrlStruct->theMotor->red = 14.0;								//je crois que c'est celui choisi
    	theCtrlStruct->theMotor->theMotor_l = (struct Motor_Ctrl *) malloc(sizeof(struct Motor_Ctrl));
    	theCtrlStruct->theMotor->theMotor_r = (struct Motor_Ctrl *) malloc(sizeof(struct Motor_Ctrl));
	
	theCtrlStruct->theMotor->theMotor_l->Ki = 0.0;
	theCtrlStruct->theMotor->theMotor_l->Kp = 0.0;
	theCtrlStruct->theMotor->theMotor_l->K = 1.0;
	theCtrlStruct->theMotor->theMotor_l->komega = 1.0;
	theCtrlStruct->theMotor->theMotor_l->integral_error = 0.0;

	theCtrlStruct->theMotor->theMotor_r->Ki = 0.0;
	theCtrlStruct->theMotor->theMotor_r->Kp = 0.0;
	theCtrlStruct->theMotor->theMotor_r->K = 1.0;
	theCtrlStruct->theMotor->theMotor_r->komega = 1.0;
	theCtrlStruct->theMotor->theMotor_r->integral_error = 0.0;

	return;
}

void run_speed_controller(CtrlStruct* theCtrlStruct, double *omega_ref){
	
    double Ua_left_out = run_motor(theCtrlStruct,theCtrlStruct->theMotor->theMotor_l,omega_ref[L_ID],theCtrlStruct->theCtrlIn->l_wheel_speed);
    double Ua_right_out = run_motor(theCtrlStruct,theCtrlStruct->theMotor->theMotor_r,omega_ref[R_ID],theCtrlStruct->theCtrlIn->r_wheel_speed);

    theCtrlStruct->theCtrlOut->wheel_commands[L_ID] = Ua_left_out * 100.0 / theCtrlStruct->theMotor->Un;
    theCtrlStruct->theCtrlOut->wheel_commands[R_ID] = Ua_right_out * 100.0 / theCtrlStruct->theMotor->Un;

    // printf("%d \n",Ua_left_out);
    // printf("%d \n",Ua_right_out);
    return;
}

double run_motor(CtrlStruct* theCtrlStruct, Motor_Ctrl* theMotor, double omega_ref, double wheel_speed){
	double motor_speed = wheel_speed / theMotor->komega * theCtrlStruct->theMotor->red;

	double delta_omega = omega_ref * theCtrlStruct->theMotor->red - motor_speed;

	double ua_p_ref = delta_omega * theMotor->Kp + theMotor->Ki * (theCtrlStruct->theMotor->dt * delta_omega + theMotor->integral_error);

	// Current limitation
	int current_sat = saturation(&ua_p_ref, theCtrlStruct->theMotor->Ra * theCtrlStruct->theMotor->In);

	double ua_ref = ua_p_ref + theCtrlStruct->theMotor->kphi * motor_speed;
	

	// Voltage limitation
	int voltage_sat = saturation(&ua_ref, theCtrlStruct->theMotor->Un);

	if (!current_sat && !voltage_sat) {
	    theMotor->integral_error = theCtrlStruct->theMotor->dt * delta_omega + theMotor->integral_error;
	}

	double ua = (double) ua_ref / theMotor->K;

	return ua;
}

int saturation(double *x, double xsat){
    if (*x > xsat) {
	    *x = xsat;
	    return 1;
	}
	else if (*x < -xsat ) {
	    *x = -xsat;
	    return 1;
	}
	else {
	    return 0;
	}
}

int size_UserStruct(){
    return sizeof(CtrlStruct);
}