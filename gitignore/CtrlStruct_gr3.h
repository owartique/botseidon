#ifndef _CTRLSTRUCT_GR3_H_
#define _CTRLSTRUCT_GR3_H_

#include "ctrl_io.h"

typedef struct Motor{

	// Controller
	double Ki;
	double Kp;
	
	double K; ///< gain of the power electronics
	
	double komega;  ///< omega gain of the sensor
	
	double integral_error;
	
} Motor;

typedef struct UserStruct{
    Motor *theMotor_l;
    Motor *theMotor_r;

    double Un; ///< V
    double In; ///< A
    double Nn; ///< tr/min

    double Ra; ///< omh	

    double R;  ///< radius of the wheel
    double kphi;  ///< kphi of the motorl
    double dt;
    double red; ///< reduction ratio
    
} UserStruct;

typedef struct CtrlStruct{
	UserStruct *theUserStruct;  ///< user defined CtrlStruct
	CtrlIn *theCtrlIn;   ///< controller inputs
	CtrlOut *theCtrlOut; ///< controller outputs
} CtrlStruct;


int size_UserStruct();

#endif //ifndef