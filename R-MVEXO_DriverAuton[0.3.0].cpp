/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Driver/Auton                */
/*                Version 0.3.0               */
/*--------------------------------------------*/

void auton(int autonMode){
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(1,0);
    ctrPrimary.Screen.print("Autonomous");
    if (autonMode == 1){
        //Declare variable here
        robotMain.Screen.clearScreen();
        robotMain.Screen.setCursor(1,0);
        int clock = 0;
        int process = 0; //variable to control where in the auton you are
        mtrDriveLeft.resetRotation();
        mtrDriveRight.resetRotation();
        driveYawPID.reset();
        driveSpeedPID.reset();
        while (confirmAuton() && process < 3){//Set process number to last process
            //Run auton implementation here
            
            /*if (driveSpeedPID.prevDerivative < 0 && driveSpeedPID.derivative >= 0){
                robotMain.Screen.newLine();
                robotMain.Screen.print("%d", clock);
                clock = 0;
            }*/
            if(process == 0){
                if (driveToPoint(360, 0)){
                    mtrDriveLeft.resetRotation();
                    mtrDriveRight.resetRotation();
                    driveYawPID.reset();
                    driveSpeedPID.reset();
                    stopAllMotors();
                    process++;
                }
            };
            if(process == 1){
                if (pointTurn(650)){
                    mtrDriveLeft.resetRotation();
                    mtrDriveRight.resetRotation();
                    driveYawPID.reset();
                    driveSpeedPID.reset();
                    stopAllMotors();
                    process++;
                    wait(150);
                }
            }
            if (process == 2){
                if (driveToPoint(360, 900)){
                    process++;
                }
                robotMain.Screen.clearScreen();
                robotMain.Screen.setCursor(1,0);
                robotMain.Screen.print("Gyro: %d", gyroNav.value(vex::analogUnits::range12bit));
                robotMain.Screen.newLine();
            }
            runDiagnostics();
            wait(20);//run at 50 Hz
            clock ++;
        }
    }
    if (autonMode == 2){
        //Declare variable here
        robotMain.Screen.clearScreen();
        robotMain.Screen.setCursor(1,0);
        int clock = 0;
        int process = 0;
        launchAnglePID.reset();
        BallLauncher targetSystem;
        while (confirmAuton() && process < 1){
            targetSystem.scanForFlags();
            double angle = targetSystem.targetSpecificFlag();
            if (angle != -1){
                if(setLauncherToAngle(angle) < 1){
                    robotMain.Screen.clearScreen();
                    robotMain.Screen.setCursor(1,0);
                    robotMain.Screen.print("%f", getAccelTiltAngle());
                }
            } else {
                mtrLauncherAngle.stop(vex::brakeType::hold);
            }
            clock++;
            wait(20);
        }
    }
}

void driver(){
    //Declare variables here
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(0,0);
    ctrPrimary.Screen.print("Party Time");
    BallLauncher targetSystem;
    int clock = 0;
    bool mode6 = true;
    while (confirmDriver()){
        //Run driver implementation here
        int lightMode = targetSystem.scanForFlags();
        double angle = targetSystem.targetSpecificFlag();
        if (ctrPrimary.ButtonR1.pressing() || ctrPrimary.ButtonR2.pressing()){
            robot.launchAngle(ctrPrimary.ButtonR1.pressing(), ctrPrimary.ButtonR2.pressing());
        } else {
            if (angle != -1 && !ctrPrimary.ButtonX.pressing()){
                if(setLauncherToAngle(angle) < 1){
                    if (lightMode == 5){
                        if (clock == 500){
                            clock = 0;
                            mode6 = !mode6;
                        }
                        if (mode6){
                            lightMode = 6;
                        }
                    }
                }
            } else {
                mtrLauncherAngle.stop(vex::brakeType::hold);
            }
        }
        clock += 20;
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
