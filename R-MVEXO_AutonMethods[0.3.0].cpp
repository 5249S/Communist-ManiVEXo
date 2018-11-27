/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Auton Methods               */
/*                Version 0.3.0               */
/*--------------------------------------------*/
RobotControl robot;
Pid driveSpeedPID;
Pid driveYawPID;
Pid launchAnglePID;
Pid visionHorizontalPID;
bool driveToPoint(float endpoint, float yaw){
    driveSpeedPID.kP = 1;
    driveSpeedPID.kI = 4.35;
    driveSpeedPID.kD = 0.153;
    driveYawPID.kP = 0.7;
    driveYawPID.kI = 0;
    driveYawPID.kD = 0.3;
    const int maxSpeed = 100;
    driveSpeedPID.setPoint = endpoint;
    driveYawPID.setPoint = yaw;
    int speed = (int)driveSpeedPID.pidCalc(mtrDriveLeft.rotation(vex::rotationUnits::deg));
    int turn = (int)driveYawPID.pidCalc(gyroNav.value(vex::analogUnits::range12bit));
    if (speed > maxSpeed){
        speed = maxSpeed;
        driveSpeedPID.resetIntegral();
    }
    if (speed < -maxSpeed){
        speed = -maxSpeed;
        driveSpeedPID.resetIntegral();
    }
    if (turn > maxSpeed){
        turn = maxSpeed;
        driveYawPID.resetIntegral();
    }
    if (turn < -maxSpeed){
        turn = -maxSpeed;
        driveYawPID.resetIntegral();
    }
    robot.driveH(speed, turn);
    return fabs((double)endpoint) < fabs((double)mtrDriveLeft.rotation(vex::rotationUnits::deg));
}
bool pointTurn(float yaw){
    const int maxSpeed = 100;
    driveYawPID.kP = 0.45;
    driveYawPID.kI = 0.7;
    driveYawPID.kD = 0.73;
    driveYawPID.setPoint = yaw;
    int turnSpeed = (int)driveYawPID.pidCalc(gyroNav.value(vex::analogUnits::range12bit));
    if (turnSpeed > maxSpeed){
        turnSpeed = maxSpeed;
        driveYawPID.resetIntegral();
    }
    if (turnSpeed < -maxSpeed){
        turnSpeed = -maxSpeed;
        driveYawPID.resetIntegral();
    }
    robot.driveH(0, turnSpeed);
    return fabs((double)yaw) < fabs((double)gyroNav.value(vex::analogUnits::range12bit));
    
}
double setLauncherToAngle(double angle){
    const int maxSpeed = 30;
    launchAnglePID.kP = 3;
    launchAnglePID.kI = 0;
    launchAnglePID.kD = 1.7;
    launchAnglePID.setPoint = angle;
    int speed = (int)launchAnglePID.pidCalc((float)getAccelTiltAngle());
    if (speed > maxSpeed){
        speed = maxSpeed;
        launchAnglePID.resetIntegral();
    }
    if (speed < -maxSpeed){
        speed = -maxSpeed;
        launchAnglePID.resetIntegral();
    }
    robot.launchAnglePower(speed);
    return fabs(angle - getAccelTiltAngle());
}
