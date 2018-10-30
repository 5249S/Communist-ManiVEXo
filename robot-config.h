//robot-config.h
vex::vision visLauncher = vex::vision(vex::PORT3);
vex::gyro gyroLauncher = vex::gyro(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.D);

vex::vision visLauncher = vex::vision(vex::PORT2);
vex::gyro gyroLauncher = vex::gyro(robotMain.ThreeWirePort.E);
vex::accelerometer accelLauncherX = vex::accelerometer(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.D);

