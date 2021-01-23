#include <cstdio>
#include <time.h>

#include <wiringPiSPI.h>
#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"
#include "IO/COM/SPI/SPI.hh"

#include <signal.h>

#include <iostream>

using namespace std;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


#define CAN_BR 125e3

int main()
{
	printf("hello world\n");
	printf("##############################################################################################################\n");
    	printf("\t\t\tWelcome to the Minibot project of the ELEME2002 class :)");
    	printf("##############################################################################################################\n");
    	printf("\t\t I'm Mister GrayCode, please take care of me !\n");
    	printf("\t\t Please do not interchange the chips on my tower/motor PWM boards !\n");
    	printf("\t\t Try to respect the C-file interface when programming me because\n \t\t it will be the same in the robotic project (Q2) !\n");


	CAN *can;
	can = new CAN(CAN_BR);
	can->configure();
	can->ctrl_led(0);

    //can->push_PropDC(30,30);
    //usleep(100);
    can->ctrl_motor(0);
    if(ctrl_c_pressed){
        can->ctrl_motor(0);
    }


}
