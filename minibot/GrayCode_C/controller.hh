#ifndef _SPEED_CONTROLLER_
#define _SPEED_CONTROLLER_

#include "IO/ctrl_io.h"

typedef struct Motor_Ctrl{

	// Controller
	double Ki;
	double Kp;
	
	double K; ///< gain of the power electronics
	
	double komega;  ///< omega gain of the sensor
	
	double integral_error;
	
} Motor_Ctrl;

typedef struct Motor{

    Motor_Ctrl *theMotor_l;
    Motor_Ctrl *theMotor_r;
    
    double Un; ///< V
    double In; ///< A
    double Nn; ///< tr/min

    double Ra; ///< omh

    double R;  ///< radius of the wheel
    double kphi;  ///< kphi of the motorl
    double dt;
    double red; ///< reduction ratio
    
} Motor;

typedef struct CtrlStruct{
	Motor *theMotor;  ///< user defined CtrlStruct
	CtrlIn *theCtrlIn;   ///< controller inputs
	CtrlOut *theCtrlOut; ///< controller outputs
} CtrlStruct;


int size_Motor_Ctrl();

void init_speed_controller(CtrlStruct* theCtrlStruct);
void run_speed_controller(CtrlStruct* theCtrlStruct, double *omega_ref);
double run_motor(CtrlStruct* theCtrlStruct, Motor_Ctrl * theMotor, double omega_ref, double wheel_speed);
int saturation(double *x, double xsat);

#endif