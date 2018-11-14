/*--------------------------------------------*/
/*                    5249S                   */
/*                The ManiVEXo                */
/*                Driver/Auton                */
/*                Version 0.3.0               */
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
            wait(20);//run at 50 Hz
        }
    }
}

void driver(){
    //Declare variables here
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(0,0);
    ctrPrimary.Screen.print("Party Time");
    gyroLauncherSet.setValues(getAccelTiltAngle(), gyroLauncher.value(vex::analogUnits::range12bit)*10, true);
    while (confirmDriver()){
        robotMain.Screen.clearScreen();
        robotMain.Screen.setCursor(1,0);
        robotMain.Screen.print("Gyro: %d", gyroLauncherSet.value(gyroLauncher.value(vex::analogUnits::range12bit)));
        robotMain.Screen.newLine();
        robotMain.Screen.print("Gyro: %d", gyroLauncher.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("AccX: %d", accelLauncherX.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("AccY: %d", accelLauncherY.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("AccZ: %d", accelLauncherZ.value(vex::analogUnits::range12bit));
        
        robotMain.Screen.print("Tilt: %f", getAccelTiltAngle());
        //Run driver implementation here
        robot.launchAngle(ctrPrimary.ButtonR1.pressing(), ctrPrimary.ButtonR2.pressing());
        robot.launchFire(ctrPrimary.ButtonX.pressing());
        robot.claw(ctrPrimary.ButtonL1.pressing(), ctrPrimary.ButtonL2.pressing());
        int y = ctrPrimary.Axis3.position(vex::percentUnits::pct);
        int x = ctrPrimary.Axis4.position(vex::percentUnits::pct);
        robot.lift(ctrPrimary.Axis2.position(vex::percentUnits::pct));
        robot.driveH(y, x);
        runDiagnostics();
        wait(20);//run at 50 Hz
    }
    
}
