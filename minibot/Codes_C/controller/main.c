#include <stdio.h>
#include <stdlib.h>
#include "CtrlStruct_gr3.h"
#include "speed_controller.h"




int main(){
    CtrlStruct *ctrl = (struct CtrlStruct*)malloc(sizeof(CtrlStruct));

    if (ctrl==NULL) {
        printf("fuck\n");
    }
    else{ printf("yes\n");

    init_speed_controller(ctrl);
    }


    printf("%f \t %f \n",ctrl->theUserStruct->Un,ctrl->theUserStruct->theMotor_l->Ki);
    return 0;
}


