//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::competition compControl;

vex::motor mtrDriveLeft = vex::motor(vex::PORT1);
vex::motor mtrDriveRight = vex::motor(vex::PORT10, true);
vex::motor mtrLiftLeft = vex::motor(vex::PORT3, true);
vex::motor mtrLiftRight = vex::motor(vex::PORT8);
vex::motor mtrClaw = vex::motor(vex::PORT4, true);
vex::motor mtrLauncherAngle = vex::motor(vex::PORT5);
vex::motor mtrLauncherFire = vex::motor(vex::PORT11);

vex::vision::signature SIG_FLAG_RED (1, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_FLAG_BLUE (2, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision visLauncher (vex::PORT7, 50, SIG_FLAG_RED, SIG_FLAG_BLUE, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
vex::gyro gyroNav = vex::gyro(robotMain.ThreeWirePort.A);

vex::accelerometer accelLauncherX = vex::accelerometer(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.D);
