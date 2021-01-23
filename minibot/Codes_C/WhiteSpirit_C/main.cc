
// Last modified Dec 15 2020 by Augustin

#include "custom.hh"
#include <signal.h>

#include <iostream>

using namespace std;

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
    usleep(50);
    can->ctrl_led(1);
    usleep(50);
    can->ctrl_motor(1);
    usleep(50);

    signal(SIGINT, ctrlc);

// Follow the beacon
    bool obs = true;
    bool* obstacle_ptr = &obs;

    bool roll = false;
    bool* rolling_ptr = &roll;

    double ang = 0.0;
    double* angle_ptr = &ang;

    string msg;

// Controller
    CtrlStruct *ctrl = (struct CtrlStruct*)malloc(sizeof(CtrlStruct));
    init_speed_controller(ctrl);
    double omega_ref[2];
    double *omega_ref_ptr = omega_ref;

    omega_ref_ptr[L_ID] = 0.0;//CRUISE_SPEED;
    omega_ref_ptr[R_ID] = 0.0;//CRUISE_SPEED;

// parcours
    double left = 0.0;
    double right = 0.0;
    double* leftSum = &left;
    double* rightSum = &right;

    int i = 0;
    int* cnt_ptr = &i;

    int f = 1;
    int* first_ptr = &f;

    int a = 0;
    int* a_ptr = &a;

    encoder(ctrl, leftSum, rightSum, spi);
    run_speed_controller(ctrl,omega_ref_ptr);
    can->push_PropDC(ctrl->theCtrlOut->wheel_commands[L_ID],ctrl->theCtrlOut->wheel_commands[R_ID]);
    usleep(100);

// site
    ofstream myfile;

    vector<double> left_speed;
    vector<double> right_speed;

    vector<double> rayon;
    vector<double> alpha;

// demo's

    myfile.open("log/demo.txt");
    myfile<<"Demo 1 \n";
    myfile.close();

    printf("Demo 1 debut \n");

    demo1(omega_ref_ptr, ctrl, leftSum, rightSum, can, spi, left_speed, right_speed);
    printf("Demo 1 fin \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 1 finie \n";
    myfile.close();

    vector<pair<string , vector<double> > > vals = {{"Motorspeed_L", left_speed}, {"Motorspeed_R", right_speed}};
    write_csv("log/speed.csv", vals);
    
    while(!ctrl_c_pressed){}

    ctrl_c_pressed = false;

    printf("Demo 2 debut \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 2 \n";
    myfile.close();

    demo2(omega_ref_ptr, ctrl, leftSum, rightSum, can, spi);
    printf("Demo 2 fin \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 2 finie \n";
    myfile.close();

    while(!ctrl_c_pressed){}

    ctrl_c_pressed = false;
    sleep(1);
    /*

    printf("Demo 3 debut \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 3 \n";
    myfile.close();

    while(!ctrl_c_pressed){
        follow(angle_ptr, obstacle_ptr,rolling_ptr, can, msg);
        myfile.open("log/demo.txt");
        myfile<<msg;
        myfile.close();
    }

    printf("Demo 3 fin \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 3 finie \n";
    myfile.close();


    stop(ctrl,omega_ref_ptr,leftSum,rightSum,can,spi);
    ctrl_c_pressed = false;

    while(!ctrl_c_pressed){}
    ctrl_c_pressed = false;

    printf("Demo 4 debut \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 4 \n";
    myfile.close();
    avoid(omega_ref_ptr, ctrl, leftSum, rightSum, can, spi, rayon, alpha);
    printf("Demo 4 fin \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 4 finie \n";
    myfile.close();

    stop(ctrl,omega_ref_ptr,leftSum,rightSum,can,spi);

    while(!ctrl_c_pressed){}

    ctrl_c_pressed = false;
    sleep(1);

    printf("Demo 5 debut \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 5 \n";
    myfile.close();
    while(!ctrl_c_pressed){
        parcours(omega_ref_ptr,ctrl, cnt_ptr,first_ptr, a_ptr, leftSum, rightSum, can, spi);
        encoder(ctrl, leftSum, rightSum, spi);
        run_speed_controller(ctrl,omega_ref_ptr);
        can->push_PropDC(ctrl->theCtrlOut->wheel_commands[L_ID],ctrl->theCtrlOut->wheel_commands[R_ID]);
        usleep(100);
        if (*cnt_ptr==18 | ctrl_c_pressed){
            omega_ref_ptr[L_ID] = 0.0;
            omega_ref_ptr[R_ID] = 0.0;
            encoder(ctrl, leftSum, rightSum, spi);
            run_speed_controller(ctrl,omega_ref_ptr);
            can->push_PropDC(ctrl->theCtrlOut->wheel_commands[L_ID],ctrl->theCtrlOut->wheel_commands[R_ID]);
            usleep(100);
            while(ctrl->theCtrlIn->l_wheel_speed!=0.0 & ctrl->theCtrlIn->l_wheel_speed!=0.0){
                encoder(ctrl, leftSum, rightSum, spi);
                run_speed_controller(ctrl,omega_ref_ptr);
                can->push_PropDC(ctrl->theCtrlOut->wheel_commands[L_ID],ctrl->theCtrlOut->wheel_commands[R_ID]);
                usleep(100);
            }
            break;
        }
    }
    printf("Demo 5 fin \n");
    myfile.open("log/demo.txt");
    myfile<<"Demo 5 finie \n";
    myfile.close();
    */
    can->ctrl_led(0);
    usleep(50);
    can->ctrl_motor(0);
    usleep(50);
    //can->push_PropDC(0,0);
    usleep(50);
    stopLidar();

    return 0;
}

