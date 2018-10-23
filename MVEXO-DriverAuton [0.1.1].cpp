/*--------------------------------------------*/
/*                    5249S                   */
/*                The ManiVEXo                */
/*                Driver/Auton                */
/*                Version 0.1.1               */
/*--------------------------------------------*/

void auton(int autonMode){
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(1,0);
    ctrPrimary.Screen.print("Autonomous");
    if (autonMode == 1){
        //Declare variable here
        int process = 0; //variable to control where in the auton you are
        while (confirmAuton() && process < 0){//Set process number to last process
            //Run auton implementation here
            if (mode == 2 && ctrPrimary.ButtonB.pressing()){
                break;//Option for quitting in test mode
            }
            wait(20);//run at 50 Hz
        }
    }
}

void driver(){
    //Declare variables here
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(0,0);
    ctrPrimary.Screen.print("Driver Control");
    DriveMethods robot;
    Lift lift;
    while (confirmDriver()){
        //Run driver implementation here
        int y = ctrPrimary.Axis3.position(vex::percentUnits::pct);
        int x = ctrPrimary.Axis4.position(vex::percentUnits::pct);
        lift.drive(ctrPrimary.Axis2.position(vex::percentUnits::pct));
        robot.driveH(y, x);
        if (mode == 3 && ctrPrimary.ButtonB.pressing()){
            break;//Option for quitting in test mode
        }
        wait(20);//run at 50 Hz
    }
    
}
