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
    if (autonMode == 1){//Run Auton 1
        //Declare variable here
        robotMain.Screen.clearScreen();
        robotMain.Screen.setCursor(1,0);
        int clock = 0;//Reset clock, motors, and process
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
            if(process == 0){//Process 0
                if (driveToPoint(360, 0)){//Wait until the robot drives 360 degrees
                    mtrDriveLeft.resetRotation();//Reset motors
                    mtrDriveRight.resetRotation();
                    driveYawPID.reset();
                    driveSpeedPID.reset();
                    stopAllMotors();
                    process++;//Move to process 1
                }
            };
            if(process == 1){//Process 1
                if (pointTurn(-650)){//Wait until the robot turns 90 degrees to the left
                    mtrDriveLeft.resetRotation();//Reset motors
                    mtrDriveRight.resetRotation();
                    driveYawPID.reset();
                    driveSpeedPID.reset();
                    stopAllMotors();
                    process++;//go to process 2
                    wait(150);//Wait 150 mSec
                }
            }
            if (process == 2){//Process 2
                if (driveToPoint(360, -900)){//Wait until the robot drives 360 degrees forward
                    process++;//go to process 3
                }
                robotMain.Screen.clearScreen();
                robotMain.Screen.setCursor(1,0);
                robotMain.Screen.print("Gyro: %d", gyroNav.value(vex::analogUnits::range12bit));
                robotMain.Screen.newLine();
            }
            runDiagnostics();//display warnings
            wait(20);//run at 50 Hz
            clock ++;
        }
    }
    if (autonMode == 2){//Aligns ball launcher
        //Declare variable here
        robotMain.Screen.clearScreen();
        robotMain.Screen.setCursor(1,0);
        int clock = 0;//Reset clock, motors, and process
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
    ctrPrimary.Screen.print("Party Time");//Party Time
    BallLauncher targetSystem;
    bool autoTarget = false;
    bool waitForReleaseA = false;
    ctrPrimary.Screen.setCursor(3,0);
    ctrPrimary.Screen.clearLine(3);
    ctrPrimary.Screen.print("AutoTarget: %s", autoTarget?"On":"Off");
    while (confirmDriver()){
        //Run driver implementation here
        if (ctrPrimary.ButtonA.pressing() && !waitForReleaseA){//Turns on auto target
            waitForReleaseA = true;
            autoTarget = !autoTarget;
            ctrPrimary.Screen.setCursor(3,0);
            ctrPrimary.Screen.clearLine(3);
            if (autoTarget){//Display auto target state
                ctrPrimary.Screen.print("AutoTarget: On");
            } else {
                ctrPrimary.Screen.print("AutoTarget: Off");
            }
        }
        if (!ctrPrimary.ButtonA.pressing() && waitForReleaseA){//Waits for release to change the state of auto target
            waitForReleaseA = false;
        }
        if (autoTarget){//Run limited chassis
            targetSystem.scanForFlags();//Run target system
            double angle = targetSystem.targetSpecificFlag();
            if (!ctrPrimary.ButtonR1.pressing() && !ctrPrimary.ButtonR2.pressing()){//If no manual controls are being pressed, Angle the ball launcher
                if (angle != -1 && !ctrPrimary.ButtonX.pressing()){
                    setLauncherToAngle(angle);
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
            }
            if (ctrPrimary.ButtonLeft.pressing()){//Run the chassis on limited controls
                robot.driveH(0,-30);
            } else {
                if (ctrPrimary.ButtonRight.pressing()){
                    robot.driveH(0,30);
                } else {
                    horizontalAlignFlag(targetSystem.closestPositionX);
                }
            }
        } else {
            int y = ctrPrimary.Axis3.position(vex::percentUnits::pct);//Run chassis with joystick
            int x = ctrPrimary.Axis4.position(vex::percentUnits::pct);
            robot.lift(ctrPrimary.Axis2.position(vex::percentUnits::pct));
            robot.driveH(y, x);
        }
        robot.launchAngle(ctrPrimary.ButtonR1.pressing(), ctrPrimary.ButtonR2.pressing());//Angle launcher manually
        robot.liftBall(ctrPrimary.ButtonUp.pressing(), ctrPrimary.ButtonDown.pressing());//Ball Lift
        robot.launchFire(ctrPrimary.ButtonX.pressing());//Fire the launcher
        robot.claw(ctrPrimary.ButtonL1.pressing(), ctrPrimary.ButtonL2.pressing());//Run the claw
        runDiagnostics();//Check for warnings and display
        robotMain.Screen.clearScreen();//Display the gyros value on the processor
        robotMain.Screen.setCursor(1,0);
        robotMain.Screen.print("Gyro: %d", gyroNav.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        wait(20);//run at 50 Hz
    }
}
