/*--------------------------------------------*/
/*                    5249S                   */
/*             Communist ManiVEXo             */
/*                Driver/Auton                */
/*                Version 0.1.0               */
/*--------------------------------------------*/

#include "robot-config.h"
#include <cmath>
    
static int mode = -1;
static bool colorRed = true;
bool confirmAuton();
bool confirmDriver();
void wait(int);

void auton(int autonMode){
    if (autonMode == 0){
        //Declare variable here
        int process = 0; //variable to control where in the auton you are
        while (confirmAuton() && process < 0){//Set process number to last process
            //Run auton implementation here
            if (mode == 2){
                //Run something for running in test mode
            }
            wait(20);//run at 50 Hz
        }
    }
}

void driver(){
    //Declare variables here
    while (confirmDriver()){
        //Run driver implementation here
        if (mode == 3){
            //Run something for running in test mode
        }
        wait(20);//run at 50 Hz
    }
    
}