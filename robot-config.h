//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::competition compControl;
vex::motor mtrDriveLeft = vex::motor(vex::PORT1);
vex::motor mtrDriveRight = vex::motor(vex::PORT10);

vex::vision visLauncher = vex::vision(2);
vex::gyro gyroLauncher = vex::gyro(robotMain.ThreeWirePort.B);
vex::accelerometer = 
