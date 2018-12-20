//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::competition compControl;

vex::motor mtrDriveLeft = vex::motor(vex::PORT10);
vex::motor mtrDriveRight = vex::motor(vex::PORT1, true);
vex::motor mtrLiftLeft = vex::motor(vex::PORT9, true);
vex::motor mtrLiftRight = vex::motor(vex::PORT2);
vex::motor mtrClaw = vex::motor(vex::PORT8, true);
vex::motor mtrLauncherAngle = vex::motor(vex::PORT20);
vex::motor mtrLauncherFire = vex::motor(vex::PORT19);
vex::motor mtrBallLift = vex::motor(vex::PORT18, true);

vex::vision::signature SIG_FLAG_RED (1, 7079, 7977, 7528, 409, 1415, 912, 8.199999809265137, 0);
vex::vision::signature SIG_FLAG_BLUE (2, -4121, -3129, -3625, 13045, 15237, 14141, 9.300000190734863, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision visLauncher (vex::PORT17, 61, SIG_FLAG_RED, SIG_FLAG_BLUE, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

vex::accelerometer accelLauncherX = vex::accelerometer(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.D);

vex::gyro gyroNav = vex::gyro(robotMain.ThreeWirePort.E);
vex::accelerometer accelNavX = vex::accelerometer(robotMain.ThreeWirePort.F);
vex::accelerometer accelNavY = vex::accelerometer(robotMain.ThreeWirePort.G);
vex::digital_out redLightRight = vex::digital_out(robotMain.ThreeWirePort.H);
