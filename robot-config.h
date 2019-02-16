//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::controller ctrSecond = vex::controller(vex::controllerType::partner);
vex::competition compControl;

vex::motor mtrDriveLeft = vex::motor(vex::PORT15, true);
vex::motor mtrDriveLeftBack = vex::motor(vex::PORT14);
vex::motor mtrDriveRight = vex::motor(vex::PORT13, true);
vex::motor mtrDriveRightBack = vex::motor(vex::PORT12);

vex::motor mtrLiftLeft = vex::motor(vex::PORT3, true);
vex::motor mtrLiftRight = vex::motor(vex::PORT4);
vex::motor mtrClaw = vex::motor(vex::PORT12, true);
vex::motor mtrLauncherAngle = vex::motor(vex::PORT10, true);
vex::motor mtrLauncherFire = vex::motor(vex::PORT8);
vex::motor mtrBallLift = vex::motor(vex::PORT5, true);

vex::vision::signature SIG_FLAG_RED (1, 10135, 10591, 10363, -649, 1, -324, 8.7, 0);
vex::vision::signature SIG_FLAG_BLUE (2, -1979, -1669, -1824, 4591, 5025, 4808, 6.2, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision visLauncher (vex::PORT11, 109, SIG_FLAG_RED, SIG_FLAG_BLUE, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

vex::limit limBallLift = vex::limit(robotMain.ThreeWirePort.A);
vex::accelerometer accelLauncherX = vex::accelerometer(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.D);

vex::gyro gyroNav = vex::gyro(robotMain.ThreeWirePort.G);
vex::accelerometer accelNavX = vex::accelerometer(robotMain.ThreeWirePort.E);
vex::accelerometer accelNavY = vex::accelerometer(robotMain.ThreeWirePort.F);
vex::digital_out redLightRight = vex::digital_out(robotMain.ThreeWirePort.H);
