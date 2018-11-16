RobotControl robot;
Pid driveSpeedPID;
Pid driveYawPID;

bool driveToPoint(float endpoint, float yaw){
    driveSpeedPID.kP = 0;
    driveSpeedPID.kI = 0;
    driveSpeedPID.kD = 0;
    driveYawPID.kP = 0;
    driveYawPID.kI = 0;
    driveYawPID.kD = 0;
    const int maxSpeed = 100;
    driveSpeedPID.setPoint = endpoint;
    driveYawPID.setPoint = yaw;
    int speed = (int)driveSpeedPID.pidCalc(mtrDriveLeft.rotation(vex::rotationUnits::rev));
    int turn = (int)driveYawPID.pidCalc(gyroNav.value(vex::analogUnits::range12bit));
    if (speed > maxSpeed){
        speed = maxSpeed;
    }
    if (speed < -maxSpeed){
        speed = -maxSpeed;
    }
    if (turn > maxSpeed){
        turn = maxSpeed;
    }
    if (turn < -maxSpeed){
        turn = -maxSpeed;
    }
    robot.driveH(speed, turn);
}
bool pointTurn(float yaw){
    const int maxSpeed = 100;
    driveYawPID.kP = 0;
    driveYawPID.kI = 0;
    driveYawPID.kD = 0;
    driveYawPID.setPoint = yaw;
    int turnSpeed = (int)driveYawPID.pidCalc(gyroNav.value(vex::analogUnits::range12bit));
    if (turnSpeed > maxSpeed){
        turnSpeed = maxSpeed;
    }
    if (turnSpeed < -maxSpeed){
        turnSpeed = -maxSpeed;
    }
    robot.driveH(0, turnSpeed);
}
