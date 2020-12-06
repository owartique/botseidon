#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

void init_speed_controller(CtrlStruct* theCtrlStruct);
void run_speed_controller(CtrlStruct* theCtrlStruct, double *omega_ref);
double run_motor(CtrlStruct* theCtrlStruct, Motor* theMotor, double omega_ref, double wheel_speed);
int saturation(double *x, double xsat);

#endif
