/*--------------------------------------------*/
/*                    5249S                   */
/*                The ManiVEXo                */
/*                Driver/Auton                */
/*                Version 0.2.0               */
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
    ctrPrimary.Screen.print("Party Time");
    DriveMethods robot;
    Lift lift;
    while (confirmDriver()){
        robotMain.Screen.clearScreen();//Displays gyro and accelerometer values
        robotMain.Screen.setCursor(1,0);
        robotMain.Screen.print("Gyro: %d", gyroLauncher.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("AccX: %d", accelLauncherX.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("AccY: %d", accelLauncherY.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("AccZ: %d", accelLauncherZ.value(vex::analogUnits::range12bit));
        //Run driver implementation here
        int y = ctrPrimary.Axis3.position(vex::percentUnits::pct);//Runs chassis off of left joystick values
        int x = ctrPrimary.Axis4.position(vex::percentUnits::pct);
        lift.drive(ctrPrimary.Axis2.position(vex::percentUnits::pct));//Runs lift off of right joystic value
        robot.driveH(y, x);
        if (mode == 3 && ctrPrimary.ButtonB.pressing()){
            break;//Option for quitting in test mode
        }
        runDiagnostics();
        wait(20);//run at 50 Hz
    }
    
}
