//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::controller ctrSecond = vex::controller(vex::controllerType::partner);
vex::competition compControl;

vex::motor mtrDriveLeft = vex::motor(vex::PORT15, true);
vex::motor mtrDriveLeftBack = vex::motor(vex::PORT14);
vex::motor mtrDriveRight = vex::motor(vex::PORT13);
vex::motor mtrDriveRightBack = vex::motor(vex::PORT12,true);

vex::motor mtrLauncherAngle = vex::motor(vex::PORT10, true);
vex::motor mtrLauncherFire = vex::motor(vex::PORT8, vex::gearSetting::ratio36_1, true);
vex::motor mtrLauncherFire2 = vex::motor(vex::PORT7, vex::gearSetting::ratio36_1);
vex::motor mtrBallLift = vex::motor(vex::PORT5, true);

vex::vision::signature SIG_1 (1, -3429, -2881, -3155, 8643, 9793, 9218, 5.9, 1);
vex::vision::signature SIG_2 (2, 7653, 8131, 7892, -717, -475, -596, 6.2, 1);
vex::vision::signature SIG_3 (3, -2921, -2677, -2799, -4501, -4247, -4374, 10, 1);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::code SIG_FLAG_BLUE (SIG_1, SIG_3 );
vex::vision::code SIG_FLAG_RED (SIG_2, SIG_3 );
vex::vision visLauncher (vex::PORT11, 83, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

vex::limit limBallLift = vex::limit(robotMain.ThreeWirePort.A);
vex::accelerometer accelLauncherX = vex::accelerometer(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.D);

vex::gyro gyroNav = vex::gyro(robotMain.ThreeWirePort.G);
