//robot-config.h

vex::brain robotMain;
vex::controller ctrPrimary = vex::controller(vex::controllerType::primary);
vex::controller ctrSecond = vex::controller(vex::controllerType::partner);
vex::competition compControl;

vex::motor mtrDriveLeft = vex::motor(vex::PORT12);
vex::motor mtrDriveRight = vex::motor(vex::PORT19, true);
vex::motor mtrLiftLeft = vex::motor(vex::PORT17, true);
vex::motor mtrLiftRight = vex::motor(vex::PORT2);
vex::motor mtrClaw = vex::motor(vex::PORT15, true);
vex::motor mtrLauncherAngle = vex::motor(vex::PORT7, true);
vex::motor mtrLauncherFire = vex::motor(vex::PORT6);
vex::motor mtrBallLift = vex::motor(vex::PORT4, true);

vex::vision::signature SIG_FLAG_RED (1, 7079, 7977, 7528, 409, 1415, 912, 8.199999809265137, 0);
vex::vision::signature SIG_FLAG_BLUE (2, -4121, -3129, -3625, 13045, 15237, 14141, 9.300000190734863, 0);
vex::vision::signature SIG_3 (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision visLauncher (vex::PORT5, 61, SIG_FLAG_RED, SIG_FLAG_BLUE, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);

vex::accelerometer accelLauncherX = vex::accelerometer(robotMain.ThreeWirePort.B);
vex::accelerometer accelLauncherY = vex::accelerometer(robotMain.ThreeWirePort.C);
vex::accelerometer accelLauncherZ = vex::accelerometer(robotMain.ThreeWirePort.D);

vex::gyro gyroNav = vex::gyro(robotMain.ThreeWirePort.G);
vex::accelerometer accelNavX = vex::accelerometer(robotMain.ThreeWirePort.E);
vex::accelerometer accelNavY = vex::accelerometer(robotMain.ThreeWirePort.F);
vex::digital_out redLightRight = vex::digital_out(robotMain.ThreeWirePort.H);


/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                    Main                    */
/*                Version 1.0.0               */
/*--------------------------------------------*/
/*      Place files in following order:       */
/*                    Main                    */
/*                   Methods                  */
/*                Ball Launcher               */
/*                Auton Methods               */
/*                Driver Auton                */
/*--------------------------------------------*/
#include "robot-config.h"//Includes config file
#include <cmath> //Includes math operations Ex: pow, sin, sqrt
    
static int mode = -1;//Mode for the robot to operate in
static bool colorRed = true;//True if on red alliance, false if on blue
void auton(int);//Declares Auton and driver functions, initialized in Driver Auton file
void driver();
static bool warning[10][2];//Array of warning statuses for warning the driver of problems with the robot
double getAccelTiltAngle(){//Method for getting the tilt angle of the ball launcher using the accelerometer
    double calibrationParam[4][3] = {{-328141E-10,361739E-8,736303E-10},{398013E-10,-469610E-10,-372480E-8},{-350956E-8,-130104E-10,-459309E-10},{7.20023,-7.29033,7.57130}};
    //12 parameter calibration matrix
    int X = accelLauncherX.value(vex::analogUnits::range12bit);//Gets analog values from all three axes
    int Y = accelLauncherY.value(vex::analogUnits::range12bit);
    int Z = accelLauncherZ.value(vex::analogUnits::range12bit);
    double measuredValues[4] = {(double)X,(double)Y,(double)Z,1};//Sets values in matrix
    double trueValues[3];//Declares true accelerometer values matrix
    //Multiplies the parameter matrix by the analog values
    for (int i = 0;i < 3; i++){
        double dotSum = 0;
        for (int j = 0; j < 4; j++){
            dotSum += measuredValues[j]*calibrationParam[j][i];
        }
        trueValues[i] = dotSum;
    }
    if (pow(trueValues[1],2)+pow(trueValues[2],2) == 0){//Prevents a divide by zero error in the next calculation
        return 90;
    }
    return (180/3.141592)*atan((trueValues[0])/(sqrt(pow(trueValues[1],2)+pow(trueValues[2],2))));//Calculates tilt angle
}
void runDiagnostics(){//Method for displaying any problems with the robot
    char warningText[10][6] = {"BatL ","BatH ","MdlH ","MdrH ","MllH ","MlrH","","","",""};//array of warning texts
    for (int i = 0; i < 10; i++){ //store the previous state of each error to check for a change
        warning[i][1] = warning[i][0];
    }
    warning[0][0] = robotMain.Battery.capacity() < 25;//Battery capacity < 25%
    warning[1][0] = robotMain.Battery.temperature() > 80; //Battery temperature > 80%
    warning[2][0] = mtrDriveLeft.temperature(vex::percentUnits::pct) > 45; //Left Drive Motor Temp >45%
    warning[3][0] = mtrDriveRight.temperature(vex::percentUnits::pct) >45; //Right Drive Motor Temp >45%
    warning[4][0] = mtrLiftLeft.temperature(vex::percentUnits::pct) > 45; //Left Lift Motor Temp >45%
    warning[5][0] = mtrLiftRight.temperature(vex::percentUnits::pct) > 45; //Right Lift Motor Temp >45%
    warning[6][0] = mtrClaw.temperature(vex::percentUnits::pct) > 45; //Claw Motor Temp >45%
    warning[7][0] = mtrLauncherAngle.temperature(vex::percentUnits::pct) >45; //Launcher Angle Motor Temp >45%
    warning[8][0] = mtrLauncherFire.temperature(vex::percentUnits::pct) > 45; //Launcher Fire Motor Temp >45%
    warning[9][0] = false;
    
    //Checks if a value has changed and needs to be updated to comply with controller screen's slow update rate
    bool update = false;
    for (int i = 0; i < 10; i++){
        if ((warning[i][0] && !warning[i][1]) || (warning[i][1] && !warning[i][0])){ //Update the display if any of the warnings have changed
            update = true; 
            break;
        }
    }
    if (update) {//Display all warnings
        ctrPrimary.Screen.clearLine(2);
        ctrPrimary.Screen.setCursor(2,0);
        int cursor = 0;
        for (int i = 0; i < 10; i++){
            if (warning[i][0]){
                ctrPrimary.Screen.print("%s ", warningText[i]);//Display all warning text in succession
                cursor += 6;
                ctrPrimary.Screen.setCursor(2,cursor);
            }
        }
    }
}
void clearDiagnostics(){//Clears the diagnostic warnings
    for (int i = 0; i < 10;i++){
        warning[i][1] = false;
    }
}
class GyroSettings {//Class used to set gyros to specific values, as they can't be changed in the current API
    private:
        int gyroBias = 0;
        int reverse = 1;
    public:
        void setValues(int trueValue, int currentValue, bool rev){//Sets proper values
            reverse = rev?-1:1;
            gyroBias = currentValue - reverse * trueValue;
        }
        int value(int currentValue){//returns true value with wanted bias value
            return reverse * (currentValue - gyroBias);
        }
};
void wait(int time){//waits a number of milliseconds
    vex::task::sleep(time);
}
GyroSettings gyroNavSet;
void calibrateGyros(){//Calibrates gyros
    ctrPrimary.Screen.clearScreen();//Display calibration message on the controller
    ctrPrimary.Screen.setCursor(1,0);
    ctrPrimary.Screen.print("Gyros Calibrating");
    ctrPrimary.Screen.newLine();
    ctrPrimary.Screen.print("Do Not Touch Robot!");
    ctrPrimary.Screen.newLine();
    ctrPrimary.Screen.print("(B) Bypass");
    gyroNav.startCalibration();
    int timer = 0;
    while(gyroNav.isCalibrating() || timer < 0){//waits for both gyros to finish or three seconds have passed
        if (ctrPrimary.ButtonB.pressing()){
            break;//allows bypass
        }
        wait(20);
        timer += 20;
    }
    while(ctrPrimary.ButtonB.pressing()){wait(20);}
    gyroNavSet.setValues(0, gyroNav.value(vex::rotationUnits::deg), false);
    ctrPrimary.Screen.clearScreen();
}
void stopAllMotors(){//stops all motors on the robot
    mtrDriveLeft.stop(vex::brakeType::coast);
    mtrDriveRight.stop(vex::brakeType::coast);
    mtrLiftLeft.stop(vex::brakeType::coast);
    mtrLiftRight.stop(vex::brakeType::coast);
    mtrClaw.stop(vex::brakeType::coast);
    mtrLauncherAngle.stop(vex::brakeType::coast);
    mtrLauncherFire.stop(vex::brakeType::coast);
	mtrBallLift.stop(vex::brakeType::coast);
    clearDiagnostics();
}
void clearMotorRotations(){
    mtrDriveLeft.resetRotation();
    mtrDriveRight.resetRotation();
    mtrLiftLeft.resetRotation();
    mtrLiftRight.resetRotation();
    mtrClaw.resetRotation();
    mtrLauncherAngle.resetRotation();
    mtrLauncherFire.resetRotation();
    mtrBallLift.resetRotation();
}
bool isField(){//Method for checking if either field control device is connected
    return compControl.isCompetitionSwitch() || compControl.isFieldControl();
}
class DisplaySelection {//Class created to hold and change the values needed to move the display up and down
        private: 
            int maxLines = 3;//Number of controller display lines
            int topLine = 0;//Choice on the top line of the controller
            int position = 0;//Position of the arrow
            unsigned int max = 0;//Max number of choices
            bool selectionMade = false;
    
            int getCurrent(){//returns the option the arrow is on
                return topLine + position;
            }
            void moveDown(){//Moves display down
                if (getCurrent() != max - 1){//If the arrow is not at the last choice, move everything down
                    if (position == maxLines - 1){//Move the options down if the arrow is at the bottom
                        topLine ++;
                    } else {//Move the arrow down otherwise
                        position ++;
                    }
                } else {//If the arrow is at the last choice, return to the top
                    topLine = 0;
                    position = 0;
                }
            }
            void moveUp(){//Moves Display up
                if (getCurrent() != 0){//If the arrow is at not at the first selection, move everything up
                    if (position == 0){//move the options up if the arrow is at the top
                        topLine --;
                    } else {//Otherwise move the arrow up
                        position --;
                    }
                } else {//If the arrow is at the first choice, go to the bottom
                    position = maxLines - 1;
                    topLine = max - maxLines;
                }
            }
        public:
            char text[8][32];//storage for text options
            DisplaySelection(unsigned int maxOptions){//Constructor
                if (maxOptions < maxLines){//Sets the maxlines to the option number in case there are less options that usable lines
                    maxLines = maxOptions;
                }
                max = maxOptions;//Set the max number of options
            }
            int select(){//returns the chosen selection
                while(true){//repeat update until a selection is chosen
                    if(ctrPrimary.ButtonA.pressing()){//Return the current number if a selection has been made
                        while(ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonUp.pressing() || ctrPrimary.ButtonDown.pressing()){wait(20);}
                        return getCurrent();
                    }
                    if(ctrPrimary.ButtonUp.pressing()){//Move up if up button is pressed
                        moveUp();
                    } 
                    if(ctrPrimary.ButtonDown.pressing()){//Move down if down button is pressed
                        moveDown();
                    }
                    ctrPrimary.Screen.clearScreen();//clears the screen
                    for (int i=0; i < maxLines; i++){//Displays lines of text based on instance variables
                        ctrPrimary.Screen.setCursor(i+1,3);//
                        ctrPrimary.Screen.print("%s", text[i + topLine]);
                    }
                    ctrPrimary.Screen.setCursor(position+1,0);
                    ctrPrimary.Screen.print("->");//Print the arrow at the position
                    while(ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonUp.pressing() || ctrPrimary.ButtonDown.pressing()){wait(20);}//wait for all buttons to be released
                    while(!(ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonUp.pressing() || ctrPrimary.ButtonDown.pressing())){//Waits for a button to be pressed to prevent controller lag
                        if (isField()){//If the robot is connected to the field, display message to remove the cable
                            ctrPrimary.Screen.clearScreen();
                            ctrPrimary.Screen.setCursor(1,0);
                            ctrPrimary.Screen.print("Remove Field Cable");
                            while (isField()){//Wait for field cable to be removed
                                wait(20);
                            }
                            break;//Break the loop to redisplay the options
                        }
                        wait(20);
                    }
                }
            }
};
bool confirmAuton(){//Confirms it is allowed to run auton
    if (mode == 0 || mode == 1){//If in field control or skills mode, the competition control will be checked
        if (compControl.isAutonomous() && compControl.isEnabled() && isField()){//return true if auton is on and the robot is enabled
            return true;
        }
        return false;//otherwise return false
    }
    if (mode == 2 && !isField() && !ctrPrimary.ButtonB.pressing()){//if in auton testing mode, always allow
        return true;
    }
    return false;//return false otherwise
}
bool confirmDriver(){//Confirms it is allowed to run driver control
    if (mode == 0 || mode == 1){//If in field control or skills mode, the competition control will be checked
        if (compControl.isDriverControl() && compControl.isEnabled() && isField()){//return true if driver is on and the robot is enabled
            return true;
        }
        return false;//otherwise return false
    }
    if (mode == 3 && !isField() && !ctrPrimary.ButtonB.pressing()){//if in driver mode, always allow
        return true;
    }
    return false;//return false otherwise
}
int selectAutonomous(){//method for selecting autons
    DisplaySelection selectAuton = DisplaySelection(8);//create display selection object
    strcpy(selectAuton.text[0], "Bypass");//place names of autons in array
    strcpy(selectAuton.text[1], "Skill12");
    strcpy(selectAuton.text[2], "Red11P");
    strcpy(selectAuton.text[3], "Red8NP");
    strcpy(selectAuton.text[4], "Blue11P");
    strcpy(selectAuton.text[5], "Blue8NP");
    strcpy(selectAuton.text[6], "Red4");
    strcpy(selectAuton.text[7], "Blue4");
    return selectAuton.select();
}
void colorSelect(){//method for selecting field color
    DisplaySelection selectColor = DisplaySelection(2);//create display object
    strcpy(selectColor.text[0], "Red");//set array values to colors
    strcpy(selectColor.text[1], "Blue");
    strcpy(selectColor.text[2], "");
    strcpy(selectColor.text[3], "");
    strcpy(selectColor.text[4], "");
    colorRed = (selectColor.select() == 0);
}
int main() {
    robotMain.Screen.setFont(vex::fontType::mono40);
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(1,0);
    while(true){
        DisplaySelection selectMode = DisplaySelection(5); //Create Display object
        strcpy(selectMode.text[0], "Field Control");//set values in array to options
        strcpy(selectMode.text[1], "Skills Control");
        strcpy(selectMode.text[2], "Auton Testing");
        strcpy(selectMode.text[3], "Driver Control");
        strcpy(selectMode.text[4], "Exit");//for closing the program
        mode = selectMode.select();
        if(mode == 0){//Field control was selected
            calibrateGyros();//Calibrate gyro sensors
            clearMotorRotations();
            colorSelect();//select team color
            int autonMode = selectAutonomous();//select auton to run
            while(true){//loop for competition
                if (!isField()){//Waits for the user to connect to the field after selections are made
                    ctrPrimary.Screen.clearScreen();
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.print("Connect to Field");
                    ctrPrimary.Screen.newLine();
                    ctrPrimary.Screen.print("(B) Close");
                    while(!ctrPrimary.ButtonB.pressing() && !isField()){wait(20);}
                    if(ctrPrimary.ButtonB.pressing()){
                        break;
                    }
                }
                while(!compControl.isEnabled()){//Wait while the robot is disabled
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.clearLine();
                    ctrPrimary.Screen.print("FC-Disabled");
                    while(!compControl.isEnabled()){wait(20);}
                }
                
                if(compControl.isEnabled() && compControl.isAutonomous()){//runs auton when enabled and autonomous
                    auton(autonMode);
                    while(compControl.isEnabled() && compControl.isAutonomous() && isField()){wait(20);}//Waits for auton to end (50 Hertz)
                }
                if(compControl.isEnabled() && compControl.isDriverControl()){//runs driver control when enabled and driver control
                    driver();
                    while(compControl.isEnabled() && compControl.isDriverControl() && isField()){wait(20);}//Waits for driver control to end (50 Hertz)
                }
                stopAllMotors();
            }
            stopAllMotors();
        }
        if(mode == 1){//Skills mode- same as field control, except no color selection
	        calibrateGyros();
            clearMotorRotations();
            colorRed = true;
            int autonMode = selectAutonomous();
            ctrPrimary.Screen.clearScreen();
            while(true){
                if (!isField()){
                    ctrPrimary.Screen.clearScreen();
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.print("Connect to Field");
                    ctrPrimary.Screen.newLine();
                    ctrPrimary.Screen.print("(B) Close");
                    while(!ctrPrimary.ButtonB.pressing() && !isField()){wait(20);}
                    if(ctrPrimary.ButtonB.pressing()){
                        break;
                    }
                }
                while(!compControl.isEnabled()){//Waits while robot is disabled
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.clearLine();
                    ctrPrimary.Screen.print("SK-Disabled");
                    while(!compControl.isEnabled()){wait(20);}
                }
                if(compControl.isEnabled() && compControl.isAutonomous()){//runs auton when enabled and autonomous
                        auton(autonMode);
                        while(compControl.isEnabled() && compControl.isAutonomous() && isField()){wait(20);}//Waits for auton to end (50 Hertz)
                }
                if(compControl.isEnabled() && compControl.isDriverControl()){//runs driver control when driver control
                    driver();
                    while(compControl.isEnabled() && compControl.isDriverControl() && isField()){wait(20);}//Waits for driver control to end (50 Hertz)
                }
                stopAllMotors();
            }
            stopAllMotors();
        }
        if(mode == 2){//Auton testing mode
            while (true){
                while(true){
                    ctrPrimary.Screen.clearScreen();
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.print("Setup Robot");
                    ctrPrimary.Screen.newLine();
                    ctrPrimary.Screen.print("(A) Done");//Waits until robot is placed properly for auton
                    ctrPrimary.Screen.newLine();
                    ctrPrimary.Screen.print("(Y) Exit");
                    while (!ctrPrimary.ButtonA.pressing() && !ctrPrimary.ButtonY.pressing()){wait(20);}
                    if (ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonY.pressing()){
                        break;
                    }
                    wait(50); //Update at 20 hertz
                }
                if (ctrPrimary.ButtonY.pressing()){
                    break;
                }
                while(ctrPrimary.ButtonA.pressing()){wait(20);}
                calibrateGyros();
                clearMotorRotations();
                int selection = selectAutonomous();
                colorSelect();//select color
                auton(selection);//run selected auton
                stopAllMotors();
                while(ctrPrimary.ButtonB.pressing()){wait(20);}//wait for exit button to be released
            }
        }
        if(mode == 3){//Runs driver control
            calibrateGyros();
            clearMotorRotations();
            driver();
            stopAllMotors();
            while(ctrPrimary.ButtonB.pressing()){wait(20);}//wait for exit button to be released
        }
        if(mode == 4){//Exits program
            return 0;
        }
    }
}
/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                   Methods                  */
/*                Version 1.0.0               */
/*--------------------------------------------*/

//The methods used in these classes will not contain while loops, to prevent linearity and to prevent freezing of autons
class Pid {
    private:
        //variables that affect pid that will remain hidden from user to increase stability/abstraction
        float prevError = 0;
        float pidIntegral = 0;
    public:
        
        float setPoint = 0;
        float kP = 0.0;
        float kI = 0.0;
        float kD = 0.0;
        float prevDerivative = 0;
        float derivative = 0;
        float pidCalc(float processVar){
            //This is a single calculation for one cycle of a PID calculator, run in while loop, put output into control system
            //Define variables
            float pidProportional = 0.0;
            float pidDerivative = 0.0;
            float pidError = 0.0;
            prevDerivative = derivative;
            //Calculate Error
            pidError = setPoint - processVar;

            //Calculate Proportional
            pidProportional = pidError * kP;
            
            //Calculate Derivative
            pidDerivative = (pidError - prevError) * kD;
            derivative = pidError - prevError;
            //Calculate Integral
            if (fabs((double)pidIntegral) < 50) {
                    pidIntegral += (pidError + prevError)/2;
            } else {
                    pidIntegral = 0;
            }

            //use pid structure to return values
            
            float adjust = pidDerivative + pidProportional + kI * pidIntegral;
            //reset error for next loop
            prevError = pidError;
            

            //return adjustment, new error, and pidIntegral
            return adjust;
        }
        void reset(){
            pidIntegral = 0;
            prevError = 0;
        }
        void resetIntegral(){
            pidIntegral = 0;
        }
};
class Launcher {//Methods for controlling the ball launcher motors
    public: 
        void launchAngle(bool up, bool down){//Run with boolean controls
            if (up && getAccelTiltAngle() < 28){
                mtrLauncherAngle.spin(vex::directionType::fwd, 30, vex::velocityUnits::pct);
            } else {
                if(down && getAccelTiltAngle() > 10){
                    mtrLauncherAngle.spin(vex::directionType::rev, 30, vex::velocityUnits::pct);
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
            }
        }
        void launchAnglePower(int power){//Run at specific power
            if (power < 0  && getAccelTiltAngle() > 10){
                mtrLauncherAngle.spin(vex::directionType::rev, (double)(-power), vex::velocityUnits::pct);
            } else {
                if (power > 0 && getAccelTiltAngle() < 28){
                    mtrLauncherAngle.spin(vex::directionType::fwd, (double)power, vex::velocityUnits::pct);
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
            }
        }
        void launchFire(bool powerFwd, bool powerRev){//Fire the ball launcher
            if (powerFwd || powerRev){
                mtrLauncherFire.spin(vex::directionType::fwd, powerFwd?100:-100, vex::velocityUnits::pct);
            } else {
                mtrLauncherFire.stop(vex::brakeType::coast);
            }
        }
};
class Claw {//Methods for running the claw
    public: 
        void claw(bool up, bool down){//Run with booleans
            if (up&&!down){
                mtrClaw.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
            } else if(down&&!up){
                mtrClaw.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
            } else {
                mtrClaw.stop(vex::brakeType::brake);
            }
        }
        void clawPower(int power){//Run at specific power
            if (power < 0){
                mtrClaw.spin(vex::directionType::rev, (double)(-power), vex::velocityUnits::pct);
            } else {
                if (power > 0){
                    mtrClaw.spin(vex::directionType::fwd, (double)power, vex::velocityUnits::pct);
                } else {
                    mtrClaw.stop(vex::brakeType::hold);
                }
            }
            
        }
        void flipClaw(bool flipped){
            mtrClaw.startRotateTo(flipped?0:180, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
        }
};
class Lift {//Methods for running the lift
    public:
        void lift(int power){//Run lift at specific power
            if (power < 0){
                mtrLiftLeft.spin(vex::directionType::rev, (double)(-power), vex::velocityUnits::pct);
                mtrLiftRight.spin(vex::directionType::rev, (double)(-power), vex::velocityUnits::pct);
            } else if (power > 0){
                mtrLiftLeft.spin(vex::directionType::fwd, (double)power, vex::velocityUnits::pct);
                mtrLiftRight.spin(vex::directionType::fwd, (double)power, vex::velocityUnits::pct);
            } else {
                mtrLiftLeft.stop(vex::brakeType::hold);
                mtrLiftRight.stop(vex::brakeType::hold);
            }
        }
        void setLiftLevel(int level, double power = 30){
            const double levels[6] = {0, 100, 420, 550, 614, 740};
            if (level > 5){
                return;
            }
            mtrLiftLeft.startRotateTo(levels[level], vex::rotationUnits::deg, power, vex::velocityUnits::pct);
            mtrLiftRight.startRotateTo(levels[level], vex::rotationUnits::deg, power, vex::velocityUnits::pct);
        }
};
class DriveMethods {
    //Class for methods for driving the robot around the field
    protected:
        //Calculators for motor power to translate remote joystick values into motor powers
        int leftMotor(int y, int x){
            int powerLeft = 0;
            if (y >= 0) {
                if (x >= 0) {
                    if (y >= x) {
                        powerLeft = y;
                    } else {
                        powerLeft = x;
                    }
                } else {
                    powerLeft = y + x;
                }
            } else {
                if (x <= 0) {
                    if (y <= x) {
                        powerLeft = y;
                    } else {
                        powerLeft = x;
                    }
                } else {
                    powerLeft = y + x;
                }
            }
            return powerLeft;
        }
        int rightMotor(int y, int x){
            return leftMotor(y, -x);
        }
    public:
    
        DriveMethods() { }
        void driveH(int y, int x){ //Method for driving the chassis with an h-drive, using turning
            //*Repeat method in while loop for continuous control, this is instant implementation for the motor power calculators*
            if (y < 3 && y > -3) {
                y = 0;
            }
            if (x < 3 && x > -3) {
                x = 0;
            }

            int leftPower = leftMotor(y, x);
            int rightPower = rightMotor(y, x);
            if (leftPower < 0){
                mtrDriveLeft.spin(vex::directionType::rev, (double)(-leftPower), vex::velocityUnits::pct);
            } else {
                mtrDriveLeft.spin(vex::directionType::fwd, (double)leftPower, vex::velocityUnits::pct);
            }
            
            if (rightPower < 0){
                mtrDriveRight.spin(vex::directionType::rev, (double)(-rightPower), vex::velocityUnits::pct);
            } else {
                mtrDriveRight.spin(vex::directionType::fwd, (double)rightPower, vex::velocityUnits::pct);
            }
            
        }
    
};
class BallLift {//Methods for controlling the ball intake
    public:
    void liftBall(bool up, bool down){//Run with boolean values
        if (up){
            mtrBallLift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        } else {
            if(down){
                mtrBallLift.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
            } else {
                mtrBallLift.stop(vex::brakeType::coast);
            }
        }
    }
};
class RobotControl: public Lift, public DriveMethods, public Claw, public Launcher, public BallLift {//Combine methods into one class
    
};
class Flag {
    private:
        const double GRAVITY = 9.8; //m/s^2 Acceleration of gravity
        const double INITIAL_VELOCITY = 22; //m/s Initial velocity of ball
        const double FLAG_HEIGHT = 0.14; //meters known height of flag object
         //Variable for holding distance
         //Variable holding height to bottom of flag
        
    public:
        bool inRange = false;
        double distance= 0;
        double height = 0;
        int xPosition = 0;
        //Equations are based on inmatic formulas
        double calculateDistance(double alpha, double beta){//Takes angle values and sets instance varibles to distance an height
            double offset = 0; //
            distance = (FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (tan(alpha));
            return height;
        }
        double calculateRequiredAngle(){//Returns angle required to hit flag in radians
            double v = INITIAL_VELOCITY;//variables to hold values, for simplicity
            double g = GRAVITY;
            double h = height + FLAG_HEIGHT/2;//height of middle of flag
            double d = distance;
            double offset = 0;//Offset varible to adjust for systematic error
            if (pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0)) < 0){
                inRange = false;
                return -10.0;
            }
            inRange = true;
            return (atan((pow(v,2.0) - sqrt(pow(v,4.0)-g*(g*pow(d,2.0)+2*h*pow(v, 2.0))))/(g*d)) + offset);//calculates angle required, casting between numbers where needed
        }
        bool checkForHit(){
            return false;
        }
};
Flag htzFlags[9];
class BallLauncher {
    private:
        
        int htzIndex = 0;//Number of flags in horizontal target zone
        const double pi = 3.141592;//Pi
        double toRad(double degrees){//Converts degrees to radians
            return degrees * (pi/180.0);
        }
        double toDeg(double radians){//converts radians to degrees
            return radians * (180.0/pi);
        }
        
        
        //The following values are for the vision sensor camera
        const double FOV = 60;//field of view
        const double FOCAL_LENGTH = 160.0/tan(toRad(FOV/2));//Focal length of the camera based on field of view 
        const int htzMax = 180;//Upper limit of horizontal target zone
        const int htzMin = 100;//Lower limit of horizontal target zone
        double angleAtPoint(int y){//calculates the verticle angle to a specific point on the camera using the field of view and focal length
            double offset = 0;//Offset for systematic error
            int yP = -y + 160;//Makes center the origin
            return (atan((yP/FOCAL_LENGTH))) + toRad(getAccelTiltAngle() + offset);//Returns angle at point
        }
    public:
        //Important note: the vision sensor is on its side, so all coordinates are inverted
        int closestPositionX = 140;
        int scanForFlags(){//
            if (colorRed){
                visLauncher.takeSnapshot(SIG_FLAG_BLUE);
            } else {
                visLauncher.takeSnapshot(SIG_FLAG_RED);
            }
            //return visLauncher.objectCount;
            htzIndex = 0;
            int shortestDistance = 200;
            closestPositionX = 140;
            for (int i = 0; i < visLauncher.objectCount; i++){//Go through each object seen and check if it is in the htz
                if (visLauncher.objects[i].centerY < htzMax && visLauncher.objects[i].centerY > htzMin){//If it is in the htz, calculate its distance and horizontal position
                    double beta = angleAtPoint(visLauncher.objects[i].originX);
                    double alpha = angleAtPoint(visLauncher.objects[i].originX + visLauncher.objects[i].width);
                    htzFlags[htzIndex].calculateDistance(alpha, beta);
                    htzIndex ++;
                    htzFlags[htzIndex].xPosition = visLauncher.objects[i].centerY;//Set the flags horizontal position
                }
            }
            if (htzIndex != 0){
                closestPositionX = 140;
                return htzIndex;
            } else {
                for (int i = 0; i < visLauncher.objectCount; i++){//Find the closest flag horizontally and align with that
                    if (abs(visLauncher.objects[i].centerY - 140) < shortestDistance){
                        shortestDistance = visLauncher.objects[i].centerY;
                    }
                }
                closestPositionX = shortestDistance;
                return 0;
            }
        }
        double targetSpecificFlag(){
            double angles[htzIndex][2];
            //return toDeg(htzFlags[0].calculateRequiredAngle());
            
            for (int i = 0; i < htzIndex; i++){//Calculate the difference between the needed angle and gyro angle for all flags in the htz
                angles[i][0] = fabs(toDeg(htzFlags[i].calculateRequiredAngle()) - getAccelTiltAngle());
                if (htzFlags[i].inRange){
                    angles[i][1] = 1;
                } else {
                    angles[i][1] = 0;
                }
            }
            int shortestAngle = -1;//set the first angle to the shortest angle
            for (int i = 0; i < htzIndex; i++){
                if (shortestAngle == -1){
                    if (angles[i][1] == 1){
                        shortestAngle = i;
                    }
                    continue;
                }
                if (angles[i][0] < angles[shortestAngle][0] && angles[i][1] == 1){//If any other angle is closer to the gyro angle, set that to the closest angle
                    shortestAngle = i;
                }
            }
            if (shortestAngle != -1){
                closestPositionX = htzFlags[shortestAngle].xPosition;//Set the horizontal alignment to the selected flag
                return toDeg(htzFlags[shortestAngle].calculateRequiredAngle());//Run the motor to reach selected angle.
            } else {
                return -1;
            }
        }
};
/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Auton Methods               */
/*                Version 1.0.0               */
/*--------------------------------------------*/
RobotControl robot;
//Create pid objects for the following methods
Pid driveSpeedPID;
Pid driveYawPID;
Pid launchAnglePID;
Pid visionHorizontalPID;
BallLauncher targetSystem;
bool driveToPoint(float endpoint, float yaw){//Drives to a specific distance in a certain direction
    driveSpeedPID.kP = 0.3;//Set gains for both pids
    driveSpeedPID.kI = 0;
    driveSpeedPID.kD = 0.050;
    driveYawPID.kP = 0;
    driveYawPID.kI = 0;
    driveYawPID.kD = 0;
    const int maxSpeed = 100;//Set motor power limit
    driveSpeedPID.setPoint = endpoint;//Set the setpoints of the pids
    driveYawPID.setPoint = yaw;
    int speed = (int)driveSpeedPID.pidCalc(mtrDriveLeft.rotation(vex::rotationUnits::deg));//Get correction values from the pids
    int turn = (int)driveYawPID.pidCalc(gyroNav.value(vex::analogUnits::range12bit));
    //If the corrections exceed the speed limit, set the speed to the limit and reset the integral
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
    robot.driveH(speed, turn);//Drive the chassis
    return fabs((double)endpoint) < fabs((double)mtrDriveLeft.rotation(vex::rotationUnits::deg));//Return whether the robot has reached the end
}
bool pointTurn(float yaw){//Method for turning the robot
    const int maxSpeed = 100;//Set gains and max speed
    driveYawPID.kP = 0.45;
    driveYawPID.kI = 0.7;
    driveYawPID.kD = 0.73;
    driveYawPID.setPoint = yaw;//Set the setpoint to the wanted turn
    int turnSpeed = (int)driveYawPID.pidCalc(gyroNav.value(vex::analogUnits::range12bit));//Calculate the correction
    //Limit the speed to the max speed in either direction and reset the integral
    if (turnSpeed > maxSpeed){
        turnSpeed = maxSpeed;
        driveYawPID.resetIntegral();
    }
    if (turnSpeed < -maxSpeed){
        turnSpeed = -maxSpeed;
        driveYawPID.resetIntegral();
    }
    robot.driveH(0, turnSpeed);
    return fabs((double)yaw) < fabs((double)gyroNav.value(vex::analogUnits::range12bit));//return whether the robot has reached the target
    
}
double setLauncherToAngle(double angle){//Sets the launcher to a specific angle based on the accelerometer
    const int maxSpeed = 30;//Set gains and max speed
    launchAnglePID.kP = 3;
    launchAnglePID.kI = 0;
    launchAnglePID.kD = 1.7;
    launchAnglePID.setPoint = angle;//Set the setpoint
    int speed = (int)launchAnglePID.pidCalc((float)getAccelTiltAngle());//Calculate correction
    //Limit the speed to the max speed in either direction
    if (speed > maxSpeed){
        speed = maxSpeed;
        launchAnglePID.resetIntegral();
    }
    if (speed < -maxSpeed){
        speed = -maxSpeed;
        launchAnglePID.resetIntegral();
    }
    robot.launchAnglePower(speed);//Power the launcher angle
    return fabs(angle - getAccelTiltAngle());//Return the current error
}
void horizontalAlignFlag(int position){//Aligns the launcher horizontally
    const int maxSpeed = 30;//Set gains and max speed
    visionHorizontalPID.kP = 2;
    visionHorizontalPID.kI = 0;
    visionHorizontalPID.kD = 1.7;
    visionHorizontalPID.setPoint = 140;//Set setpoint
    int speed = -(int)visionHorizontalPID.pidCalc((float)position);//Calculate correction
    //Limit speed in both directions
    if (speed > maxSpeed){
        speed = maxSpeed;
        visionHorizontalPID.resetIntegral();
    }
    if (speed < -maxSpeed){
        speed = -maxSpeed;
        visionHorizontalPID.resetIntegral();
    }
    robot.driveH(0, speed);//Power chassis
}
void driveForward(double tiles, double velocity){
    double leftRotation = mtrDriveLeft.rotation(vex::rotationUnits::deg) + tiles * 652;
    double rightRotation = mtrDriveRight.rotation(vex::rotationUnits::deg) + tiles * 652;
    mtrDriveLeft.startRotateTo(leftRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
    mtrDriveRight.startRotateTo(rightRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
}
void driveTurn(double degrees, double velocity){
    double currentHeading = (mtrDriveLeft.rotation(vex::rotationUnits::deg)-mtrDriveRight.rotation(vex::rotationUnits::deg)) * 90/684;
    double headingChange = degrees - currentHeading;
    double leftRotation = mtrDriveLeft.rotation(vex::rotationUnits::deg) + headingChange * 342/90;
    double rightRotation = mtrDriveRight.rotation(vex::rotationUnits::deg) - headingChange * 342/90;
    mtrDriveLeft.startRotateTo(leftRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
    mtrDriveRight.startRotateTo(rightRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
}
/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Driver/Auton                */
/*                Version 1.0.0               */
/*--------------------------------------------*/

void auton(int autonMode){
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(1,0);
    ctrPrimary.Screen.print("Autonomous");
    if (autonMode == 1){//Run Auton 1
        //Declare variable here
        int clock = 0;//Reset clock, motors, and process
        int process = 0; //variable to control where in the auton you are
        int nextProcess = 0;
        driveYawPID.reset();
        driveSpeedPID.reset();
        while (confirmAuton() && process < 28){//Set process number to last process
            switch (process){
                case -1://Case for pausing the auton until the drive motors are not moving, to allow for a timing system between processes
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        clock = 0;
                        process = nextProcess;
                    }
                    break;
                case 0://Drive forward slightly
                    robot.setLiftLevel(0);//Hold the lift down
                    driveForward(-0.1, 80);
                    robot.flipClaw(false);//Release claw
                    process = -1;
                    nextProcess = 1;
                    break;
                case 1://Turn right 45°
                    if (clock >= 100){
                        driveTurn(45, 80);
                        process = -1;
                        nextProcess = 2;
                    }
                    break;
                case 2://Drive forward to pick up the cap
                    if (clock >= 100){
                        driveForward(-0.796, 60);
                        process ++;
                    }
                    break;
                case 3://Drive forward slowly to ensure the cap is in the claw
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        driveForward(-0.25, 20);
                        process = -1;
                        nextProcess = 4;
                    }
                    break;
                case 4://Lift the lift slightly
                    if (clock >= 100){
                        robot.setLiftLevel(1, 15);
                        clock = 0;
                        process ++;
                    }
                    break;
                case 5://drive backwards
                    if (clock >= 250){
                        driveForward(0.8, 30);
                        process = -1;
                        nextProcess = 6;
                    }
                    break;
                case 6://Turn around facing the back of the field
                    if(clock >= 100){
                        driveTurn(180, 25);
                        process = -1;
                        nextProcess = 7;
                    }
                    break;
                case 7://Drive forward to line up with the pole
                    if(clock >= 100){
                        driveForward(-1.03, 40);
                        process = -1;
                        nextProcess = 8;
                    }
                    break;
                case 8://Turn to face the pole
                    if(clock >= 100){
                        driveTurn(270, 25);
                        process ++;
                    }
                    break;
                case 9://Flip the cap and lift the lift up full
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.flipClaw(true);
                        robot.setLiftLevel(5);
                        process ++;
                    }
                    break;
                case 10://Drive forward over the pole
                    if (!mtrLiftLeft.isSpinning() && !mtrLiftRight.isSpinning()){
                        driveForward(-0.233, 30);
                        process ++;
                    }
                    break;
                case 11://Place the cap on the pole
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.setLiftLevel(4);
                        process ++;
                    }
                    break;
                case 12://Backup away from the pole
                    if (!mtrLiftLeft.isSpinning() && !mtrLiftRight.isSpinning()){
                        driveForward(0.108, 30);
                        process ++;
                    }
                    break;
                case 13://Lower lift and turn towards flag
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.setLiftLevel(0);
                        driveTurn(175, 60);
                        process = -1;
                        nextProcess = 14;
                    }
                    break;
                case 14://wait 100 milliseconds
                    if (clock >= 100){
                        process ++;
                    }
                    break;//process 15 and 16 are outside of the switch statement
                case 17://drive forward to hit the flag
                    driveForward(2.5, 100);
                    process ++;
                    break;
                case 18://backup
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        driveForward(-2, 100);
                        process = -1;
                        nextProcess = 19;
                    }
                    break;
                case 19://Turn towards cap propped on ball
                    if (clock >= 100){
                        driveTurn(90, 80);
                        process = -1;
                        nextProcess = 20;
                    }
                    break;
                case 20://Drive towards cap propped on ball
                    if (clock >= 100){
                        driveForward(-1, 80);
                        process ++;
                    }
                    break;
                case 21://Lift the lift slightly at full speed to flip the cap
                    if(!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.setLiftLevel(1, 100);
                        process ++;
                    }
                    break;
                case 22://Begin driving to line up with platforms
                    if(!mtrLiftLeft.isSpinning() && !mtrLiftRight.isSpinning()){
                        driveForward(1, 80);
                        process = -1;
                        nextProcess = 23;
                    }
                    break;
                case 23://Turn facing the back
                    if (clock >= 100){
                        driveTurn(0, 80);
                        process = -1;
                        nextProcess = 24;
                    }
                    break;
                case 24://Line up with platforms
                    if (clock >= 100){
                        driveForward(1, 100);
                        process = -1;
                        nextProcess = 25;
                    }
                    break;
                case 25://Turn towards platforms
                    if (clock >= 100){
                        driveTurn(90, 80);
                        process = -1;
                        nextProcess = 26;
                    }
                    break;
                case 26://Drive onto the platforms
                    if (clock >= 100){
                        driveForward(-2, 100);
                        process ++;
                    }
                    break;
                case 27://End auton when finished
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        process ++;
                    }
                    break;
            }
            if (process == 15 || process == 16){//Fire ball
                targetSystem.scanForFlags();//Run target system
                double angle = targetSystem.targetSpecificFlag();
                if (angle != -1){
                    setLauncherToAngle(angle);                    
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
                if (process == 15 && clock >= 3100){//wait 3 seconds and then fire
                    mtrLauncherFire.startRotateTo(1800, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                    process ++;
                }
                if (process == 16 && !mtrLauncherFire.isSpinning()){//wait for the motor to completely fire
                    process ++;
                }
            }
            wait(20);
            clock += 20;
        }
    }
    if (autonMode == 2 || autonMode == 3 || autonMode == 4 || autonMode == 5){//If the auton is for the game
        //Declare variable here
        int clock = 0;//Reset clock, motors, and process
        int process = 0; //variable to control where in the auton you are
        int nextProcess = 0;
        driveYawPID.reset();
        driveSpeedPID.reset();
        while (confirmAuton() && process < 21){//Parks if auton 2 or 4 was chosen, otherwise doesnt park
            switch (process){
                case -1://Case for pausing the auton until the drive motors are not moving, to allow for a timing system between processes
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        clock = 0;
                        process = nextProcess;
                    }
                    break;
                case 0://Hold the lift down and drive forward sightly and release the claw
                    robot.setLiftLevel(0);
                    robot.liftBall(true, false);
                    driveForward(1.75, 100);
                    process = -1;
                    nextProcess = 1;
                    break;
                case 1://Turn towards the far cap
                    if (clock >= 1000){
                        driveForward(-1.65, 80);
                        process = -1;
                        nextProcess = 2;
                    }
                    break;
                case 2://Drive towards the cap
                    if (clock >= 100){
                        driveTurn(-95 * (autonMode == 2 || autonMode == 3?1:-1), 80);
                        process ++;
                    }
                    break;
                case 3://drive forwards slowly
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.launchAngle(false, true);
                        process ++;
                        clock = 0;
                    }
                    break;
                case 4://Lift the lift up slightly and flip the cap
                    if (clock >= 300){
                        robot.launchAngle(false, false);
                        process ++;
                    }
                    break;
                case 7:
                    if (clock >= 100){
                        driveForward(-0.5, 80);
                        process ++;
                    }
                    break;
                case 10: if (clock >= 100){
                    if (clock >= 100){
                        driveForward(0.5, 80);
                        process = -1;
                        nextProcess = 11;
                    }
                    break;
                }
                case 11://Wait 100 milliseconds before firing
                    if(clock >= 100){
                        robot.liftBall(false, false);
                        driveTurn(-100 * (autonMode == 2 || autonMode == 3?1:-1), 70);
                        process = -1;
                        nextProcess = 12;
                    }
                    break;
                case 12://Drive forwards to hit the bottom flag
                    if (clock >= 100){
                        driveForward(1.9, 100);
                        process ++;
                    }
                    break;
                case 13://Drive backwards
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        driveForward(-1.9, 100);
                        process = -1;
                        nextProcess = 12;
                    }
                    break;
                case 14://Turn to face the oppisite side
                    if (clock >= 100){
                        driveTurn(-90 * (autonMode == 2 || autonMode == 3?1:-1), 100);
                        robot.setLiftLevel(1);
                        process = -1;
                        nextProcess = 13;
                    }
                    break;
                case 15:
                    if (clock >= 100){
                        driveForward(-2, 100);
                        process = -1;
                        nextProcess = 14;
                    }
                case 16://Drive forwards to line up with the platform
                    if (clock >= 100){
                        driveTurn(-180 * (autonMode == 2 || autonMode == 3?1:-1), 80);
                        process = -1;
                        nextProcess = 15;
                    }
                    break;
                case 17://turn towards the platform
                    if (clock >= 100){
                        driveForward(-1.5, 100);
                        process ++;
                    }
                    break;
                case 18://Wait for the motors to stop and then end auton
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        process ++;
                    }
            }
            if (process == 5 || process == 6 || process == 8 || process == 9){//Line up and fire the ball launcher
                targetSystem.scanForFlags();//Run target system
                double angle = targetSystem.targetSpecificFlag();
                if (angle != -1){
                    setLauncherToAngle(angle);                    
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
                if ((process == 5 || process == 8) && clock >= 1600){//Wait three seconds before firing
                    mtrLauncherFire.startRotateTo(1800 + mtrLauncherFire.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                    process ++;
                    clock = 0;
                }
                if ((process == 6 || process == 9) && !mtrLauncherFire.isSpinning()){//Wait for the fire motor to stop
                    process ++;
                    clock = 0;
                }
                if (process == 6){
                    robot.liftBall(true, false);
                }
            }
            wait(20);
            clock += 20;
        }
    }
    if (autonMode == 6 || autonMode == 7){
        //Declare variable here
        int clock = 0;//Reset clock, motors, and process
        int process = 0; //variable to control where in the auton you are
        int nextProcess = 0;
        driveYawPID.reset();
        driveSpeedPID.reset();
        while (confirmAuton() && process < 12){
            switch (process){
                case -1://Case for pausing the auton until the drive motors are not moving, to allow for a timing system between processes
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        clock = 0;
                        process = nextProcess;
                    }
                    break;
                case 0://Drive forwards towards the propped cap at full speed to knock the core off of the ball and release the claw
                    robot.setLiftLevel(0);
                    robot.flipClaw(false);
                    driveForward(-1.25, 100);
                    process ++;
                    break;
                case 1://Drive backwards and raise the lift slightly
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        driveForward(0.25, 80);
                        robot.setLiftLevel(1);
                        process = -1;
                        nextProcess = 2;
                    }
                    break;
                case 2://turn so launcher side is facing the back
                    if (clock >= 100){
                        driveTurn(-90 * (autonMode == 6?1:-1), 80);
                        process = -1;
                        nextProcess = 3;
                    }
                    break;
                case 3://Drive forwards to line up with cap
                    if (clock >= 100){
                        driveForward(1, 80);
                        process = -1;
                        nextProcess = 4;
                    }
                    break;
                case 4://Turn towards cap
                    if (clock >= 100){
                        driveTurn(0, 80);
                        robot.setLiftLevel(0);
                        process = -1;
                        nextProcess = 5;
                    }
                    break;
                case 5://Drive towards the cap slowly
                    if (clock >= 100){
                        driveForward(-1, 40);
                        process = -1;
                        nextProcess = 6;
                    }
                    break;
                case 6://Flip the cap and lift the lift up
                    if (clock >= 100){
                        robot.setLiftLevel(3);
                        robot.flipClaw(true);
                        clock = 0;
                        process ++;
                    }
                    break;
                case 7://Drive backwards towards the post
                    if (clock >= 500){
                        driveForward(1, 25);
                        process = -1;
                        nextProcess = 8;
                    }
                    break;
                case 8://Turn towards the post
                    if (clock >= 100){
                        driveTurn(90 * (autonMode == 6?1:-1), 25);
                        process ++;
                    }
                    break;
                case 9://Lower the cap onto the pole
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.setLiftLevel(2);
                        process ++;
                    }
                    break;
                case 10://Backup
                    if (!mtrLiftLeft.isSpinning() && !mtrLiftRight.isSpinning()){
                        driveForward(0.2, 80);
                        process ++;
                    }
                    break;
                case 11://Wait for motion to stop and end auton
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        process ++;
                    }
                    break;
            }
            wait(20);
            clock += 20;
        }
    }
}

void driver(){
    //Declare variables here
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(0,0);
    ctrPrimary.Screen.print("Party Time");//Party Time
    bool liftMode = false;
    bool waitForReleaseA = false;
    while (confirmDriver()){
        //Run driver implementation here
        if (ctrPrimary.ButtonA.pressing() && !waitForReleaseA){//Turns on auto target
            waitForReleaseA = true;
            liftMode = !liftMode;
        }
        if (!ctrPrimary.ButtonA.pressing() && waitForReleaseA){//Waits for release to change the state of auto target
            waitForReleaseA = false;
        }
        targetSystem.scanForFlags();//Run target system
        double angle = targetSystem.targetSpecificFlag();
        if (!(ctrPrimary.ButtonR1.pressing() || ctrSecond.ButtonR1.pressing()) && !(ctrPrimary.ButtonR2.pressing() || ctrSecond.ButtonR2.pressing())){//If no manual controls are being pressed, Angle the ball launcher
            if (angle != -1 && !(ctrPrimary.ButtonX.pressing() || ctrSecond.ButtonX.pressing())){
                setLauncherToAngle(angle);
            } else {
                mtrLauncherAngle.stop(vex::brakeType::hold);
            }
        } else {
            robot.launchAngle(ctrPrimary.ButtonR1.pressing() || ctrSecond.ButtonR1.pressing(), ctrPrimary.ButtonR2.pressing() || ctrSecond.ButtonR2.pressing());//Angle launcher manually
        }
        if (!ctrPrimary.ButtonLeft.pressing() && waitForReleaseLeft){
            waitForReleaseLeft = false;
        }
        //Run the chassis
        int y = ctrPrimary.Axis3.position(vex::percentUnits::pct) * (liftMode?-1:1);//Run chassis with joystick, reverse the direction if the direction is reversed
        int x = ctrPrimary.Axis4.position(vex::percentUnits::pct);
        if (ctrSecond.ButtonLeft.pressing()){//If the second controller pressing the left or right button, turn the robot slowly
            x = -30;
        }
        if (ctrSecond.ButtonRight.pressing()){
            x = 30;
        }
        robot.driveH(y, x);//Power the lift
        robot.liftBall(ctrPrimary.ButtonL1.pressing() || ctrSecond.ButtonL1.pressing(), ctrPrimary.ButtonL2.pressing() || ctrSecond.ButtonL2.pressing());//Ball Lift
        robot.launchFire(ctrPrimary.ButtonX.pressing() || ctrSecond.ButtonX.pressing(), ctrPrimary.ButtonY.pressing() || ctrSecond.ButtonY.pressing());//Fire the launcher
        runDiagnostics();//Check for warnings and display
        robotMain.Screen.clearScreen();//Display the gyros value on the processor
        robotMain.Screen.setCursor(1,0);
        robotMain.Screen.print("Gyro: %d", gyroNav.value(vex::analogUnits::range12bit));
        robotMain.Screen.newLine();
        robotMain.Screen.print("Left: %f", getAccelTiltAngle());
        wait(20);//run at 50 Hz
    }
}