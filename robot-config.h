//robot-config.h

vex::brain = robotMain;
vex::controller ctrPrimary = vex::controller(primary);
vex::competition compControl;
vex::motor mtrDriveLeft = vex::motor(vex::PORT1);
vex::motor mtrDriveRight = vex::motor(vex::PORT10);
