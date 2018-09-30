//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::competition compControl;

vex::motor mtrDriveLeft = vex::motor(vex::PORT1);
vex::motor mtrDriveRight = vex::motor(vex::PORT10);
vex::gyro gyroNav = vex::gyro(robotMain.ThreeWirePort.A);

vex::vision visLauncher = vex::vision(vex::PORT2);
vex::gyro gyroLauncher = vex::gyro(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.D);
