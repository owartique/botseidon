
// Last modified Dec 13 2020 by Olivier

#include "customOli.hh"
#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


CAN *can = new CAN(CAN_BR);             // pointer to CAN
SPI_DE0 *spi = new SPI_DE0(0,500000);   // pointer to SPI


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
    lidarConfigure();
    can->configure();
    usleep(1000);
    can->ctrl_led(1);
    usleep(1000);
    //can->ctrl_motor(0);
    usleep(1000);  

    signal(SIGINT, ctrlc);

    // Controller
    CtrlStruct *ctrl = (struct CtrlStruct*)malloc(sizeof(CtrlStruct));
    init_speed_controller(ctrl);
    double omega_ref[2];
    double *omega_ref_ptr = omega_ref;

    omega_ref_ptr[L_ID] = 1.0;//CRUISE_SPEED; 
    omega_ref_ptr[R_ID] = 1.0;//CRUISE_SPEED;

    double left = 0.0;
    double right = 0.0;
    double* leftSum = &left;
    double* rightSum = &right;

    double ang = 0.0;
    double* angle_ptr = &ang;

    bool stopped = true;
    bool* stop_ptr = &stopped;

    bool obs = true;
    bool* obstacle_ptr = &obs;

    bool roll = false;
    bool* rolling_ptr = &roll;

    double distance = 0.30;

    can->ctrl_motor(0);

    while(1){ 

        //follow(angle_ptr, obstacle_ptr,rolling_ptr, can);
        //usleep(1000);

        line(distance, omega_ref_ptr, ctrl, leftSum, rightSum, can, spi);

        break;

        if (ctrl_c_pressed){ 
            //stop(ctrl, omega_ref_ptr, leftSum, rightSum, can, spi);
            break;
        }
    }

    can->ctrl_led(0);
    usleep(1000);
    can->push_PropDC(0,0);
    usleep(1000);
    can->ctrl_motor(0);
    stopLidar();
    
    return 0;
}

