/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                    Main                    */
/*                Version 1.1.0               */
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
    char warningText[10][6] = {"BatL ","BatH ","MdlH ","MdrH ","MblH ","MbrH","Mlf2 ","Mla ","Mlf",""};//array of warning texts
    for (int i = 0; i < 10; i++){ //store the previous state of each error to check for a change
        warning[i][1] = warning[i][0];
    }
    warning[0][0] = robotMain.Battery.capacity() < 25;//Battery capacity < 25%
    warning[1][0] = robotMain.Battery.temperature() > 80; //Battery temperature > 80%
    warning[2][0] = mtrDriveLeft.temperature(vex::percentUnits::pct) > 45; //Left Drive Motor Temp >45%
    warning[3][0] = mtrDriveRight.temperature(vex::percentUnits::pct) >45; //Right Drive Motor Temp >45%
    warning[4][0] = mtrDriveLeftBack.temperature(vex::percentUnits::pct) > 45; //Left Lift Motor Temp >45%
    warning[5][0] = mtrDriveRightBack.temperature(vex::percentUnits::pct) > 45; //Right Lift Motor Temp >45%
    warning[6][0] = mtrLauncherFire2.temperature(vex::percentUnits::pct) > 45; //Claw Motor Temp >45%
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
    mtrDriveLeftBack.stop(vex::brakeType::coast);
    mtrDriveRightBack.stop(vex::brakeType::coast);
    mtrLauncherAngle.stop(vex::brakeType::coast);
    mtrLauncherFire.stop(vex::brakeType::coast);
    mtrLauncherFire2.stop(vex::brakeType::coast);
	mtrBallLift.stop(vex::brakeType::coast);
    clearDiagnostics();
}
void clearMotorRotations(){
    mtrDriveLeft.resetRotation();
    mtrDriveRight.resetRotation();
    mtrDriveLeftBack.resetRotation();
    mtrDriveRightBack.resetRotation();
    mtrLauncherAngle.resetRotation();
    mtrLauncherFire.resetRotation();
    mtrLauncherFire2.resetRotation();
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
    DisplaySelection selectAuton = DisplaySelection(6);//create display selection object
    strcpy(selectAuton.text[0], "Bypass");//place names of autons in array
    strcpy(selectAuton.text[1], "Game12P");
    strcpy(selectAuton.text[2], "Game11NP");
    strcpy(selectAuton.text[3], "Game12BackP");
    strcpy(selectAuton.text[4], "Game9BackNP");
    strcpy(selectAuton.text[5], "Skills11");
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
/*                Version 1.1.0               */
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
            if (up && getAccelTiltAngle() < 48){
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
                if (power > 0 && getAccelTiltAngle() < 48){
                    mtrLauncherAngle.spin(vex::directionType::fwd, (double)power, vex::velocityUnits::pct);
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
            }
        }
        void launchFire(bool powerFwd, bool powerRev){//Fire the ball launcher
            if (powerFwd || powerRev){
                mtrLauncherFire.spin(vex::directionType::fwd, powerFwd?100:-100, vex::velocityUnits::pct);
                mtrLauncherFire2.spin(vex::directionType::fwd, powerFwd?100:-100, vex::velocityUnits::pct);
            } else {
                mtrLauncherFire.stop(vex::brakeType::hold);
                mtrLauncherFire2.stop(vex::brakeType::hold);
            }
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
            if (y < 2 && y > -2) {
                y = 0;
            }
            if (x < 2 && x > -2) {
                x = 0;
            }

            int leftPower = leftMotor(y, x);
            int rightPower = rightMotor(y, x);
            if (leftPower < 0){
                mtrDriveLeft.spin(vex::directionType::rev, (double)(-leftPower), vex::velocityUnits::pct);
		        mtrDriveLeftBack.spin(vex::directionType::rev, (double)(-leftPower), vex::velocityUnits::pct);
            } else {
                mtrDriveLeft.spin(vex::directionType::fwd, (double)leftPower, vex::velocityUnits::pct);
		        mtrDriveLeftBack.spin(vex::directionType::fwd, (double)leftPower, vex::velocityUnits::pct);
            }
            
            if (rightPower < 0){
                mtrDriveRight.spin(vex::directionType::rev, (double)(-rightPower), vex::velocityUnits::pct);
		mtrDriveRightBack.spin(vex::directionType::rev, (double)(-rightPower), vex::velocityUnits::pct);
            } else {
                mtrDriveRight.spin(vex::directionType::fwd, (double)rightPower, vex::velocityUnits::pct);
		mtrDriveRightBack.spin(vex::directionType::fwd, (double)rightPower, vex::velocityUnits::pct);
            }
            
        }
    
};
class BallLift {//Methods for controlling the ball intake
    public:
    void liftBall(bool up, bool down, bool limit = false){//Run with boolean values
        if (up){
            if (limit){
                    if (limBallLift.pressing()){
                    mtrBallLift.stop(vex::brakeType::coast);
                    return;
                }
            }
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
class RobotControl: public DriveMethods, public Launcher, public BallLift {//Combine methods into one class
    
};
class Flag {
    protected:
        const double GRAVITY = 9.8; //m/s^2 Acceleration of gravity
        const double INITIAL_VELOCITY = 5.47; //m/s Initial velocity of ball
        const double FLAG_HEIGHT = 0.14; //meters known height of flag object
        const double pi = 3.141592;//Pi
        const double toRad = pi/180.0;
        const double toDeg = 180.0/pi;
        struct FlagDistanceY{
            double distance = 0;
            double height = 0;
        };
        FlagDistanceY calculateDistance(double alpha, double beta){//Takes angle values and sets instance varibles to distance an height
            FlagDistanceY retFlag;
            double offset = 0; //
            retFlag.distance = (FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            retFlag.height = retFlag.distance * (tan(alpha));
            return retFlag;
        }
        struct InRangeAngle {
            bool inRange = false;
            double requiredAngle = 0;
        };
        InRangeAngle calculateRequiredAngle(double distance, double height){//Returns angle required to hit flag in radians
            InRangeAngle retAngle;
            double v = INITIAL_VELOCITY;//variables to hold values, for simplicity
            double g = GRAVITY;
            double h = height + FLAG_HEIGHT/2;//height of middle of flag
            double d = distance;
            double offset = 0;//Offset varible to adjust for systematic error
            if (pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0)) < 0){
                retAngle.inRange = false;
                retAngle.requiredAngle = -1.0;
            }
            retAngle.inRange = true;
            retAngle.requiredAngle = atan((pow(v,2.0) - sqrt(pow(v,4.0)-g*(g*pow(d,2.0)+2*h*pow(v, 2.0))))/(g*d)) * toDeg + offset;//calculates angle required, casting between numbers where needed
            return retAngle;
        }
        bool checkForHit(double distance, double height){
            double theta = toRad * getAccelTiltAngle();
            double heightAtDistance = -GRAVITY * pow(distance, 2.0) / (2 * pow(INITIAL_VELOCITY, 2.0) * pow(cos(theta), 2.0)) + distance * tan(theta);
            if (heightAtDistance > height && heightAtDistance < height + FLAG_HEIGHT){
                return true;
            } else {
                return false;
            }
        }
};
class BallLauncher: private Flag {
    private:
        //The following values are for the vision sensor camera
        const double FOV = 60;//field of view
        const double FOCAL_LENGTH = 160.0/tan(toRad * FOV/2);//Focal length of the camera based on field of view 
        double angleAtPoint(int y){//calculates the verticle angle to a specific point on the camera using the field of view and focal length
            double offset = 0;//Offset for systematic error
            int yP = -y + 160;//Makes center the origin
            return atan((yP/FOCAL_LENGTH)) + toRad * (getAccelTiltAngle() + offset);//Returns angle at point in radians
        }
        const int lineUpColors[3][3] = {{255, 0 , 0}, {0, 0, 255}, {0, 255, 0}};
    public:
        struct TargetInformation {
            int imageLocationX = 0;
            int imageLocationY = 0;
            double distance = 0;
            double height = 0;
            double requiredAngle = 0;
            bool inRange = false;
            bool willHit = false;
            int inLineX = 0;
            double inLineY = 0;
        };
        TargetInformation targetArray[20];
        int maxIndex = 20;
        void scanForFlags(){
            if (colorRed){
                visLauncher.takeSnapshot(SIG_FLAG_BLUE);
            } else {
                visLauncher.takeSnapshot(SIG_FLAG_RED);
            }
            maxIndex = visLauncher.objectCount;
            for (int i = 0; i < visLauncher.objectCount; i++){//Go through each object seen and set values
                targetArray[i].imageLocationX = visLauncher.objects[i].centerY;
                targetArray[i].imageLocationY = visLauncher.objects[i].centerX;
                double beta = angleAtPoint(visLauncher.objects[i].originX);
                double alpha = angleAtPoint(visLauncher.objects[i].originX + visLauncher.objects[i].width);
                FlagDistanceY calculatedDist = calculateDistance(alpha, beta);
                targetArray[i].distance = calculatedDist.distance;
                targetArray[i].height = calculatedDist.height;
                InRangeAngle rangeAngle = calculateRequiredAngle(calculatedDist.distance, calculatedDist.height);
                targetArray[i].requiredAngle = rangeAngle.requiredAngle;
                targetArray[i].inRange = rangeAngle.inRange;
                targetArray[i].willHit = checkForHit(calculatedDist.distance, calculatedDist.height);
                targetArray[i].inLineX = targetArray[i].imageLocationX - 140;
                targetArray[i].inLineY = targetArray[i].requiredAngle - getAccelTiltAngle();
            }
        }
        void displayInformation(){
            robotMain.Screen.clearScreen();
            robotMain.Screen.setOrigin(240, 135);
            robotMain.Screen.setPenWidth(5);
            if(!colorRed){
                robotMain.Screen.setPenColor(vex::color::red);
            } else {
                robotMain.Screen.setPenColor(vex::color::blue);
            }
            int state = 0;
            for (int i = 0; i < maxIndex; i++){
                int x = targetArray[i].inLineX;
                int y = (int)targetArray[i].inLineY * 3;
                robotMain.Screen.drawRectangle(x - 15, y - 10, 30, 20, (!colorRed?vex::color::red:vex::color::blue));
                int stateInstance = 0;
                if (abs(targetArray[i].inLineX) < 20){
                    stateInstance = 1;
                    if (fabs(targetArray[i].inLineY) < 2){
                        stateInstance = 2;
                    }
                }
                if (stateInstance > state){
                    state = stateInstance;
                }
            }
            visLauncher.setLedMode(vex::vision::ledMode::manual);
            visLauncher.setLedColor(lineUpColors[state][0], lineUpColors[state][1], lineUpColors[state][2]);
            robotMain.Screen.setPenColor(vex::color::white);
            robotMain.Screen.drawLine(-240, 0, 240, 0);
            robotMain.Screen.drawLine(-30, -10, -25, -10);
            robotMain.Screen.drawLine(-30, -10, -30, -5);
            robotMain.Screen.drawLine(25, -10, 30, -10);
            robotMain.Screen.drawLine(30, -10, 30, -5);
            robotMain.Screen.drawLine(-30, 10, -25, 10);
            robotMain.Screen.drawLine(-30, 5, -30, 10);
            robotMain.Screen.drawLine(25, 10, 30, 10);
            robotMain.Screen.drawLine(30, 5, 30, 10);
            robotMain.Screen.setOrigin(0, 135);
            robotMain.Screen.setCursor(1, 0);
            robotMain.Screen.print("%d", (int)getAccelTiltAngle());
        }
        double targetSpecificFlag(){
            int selectedIndex = -1;
            for (int i = 0; i < maxIndex; i++){
                if(selectedIndex == -1 && targetArray[i].inRange){
                    selectedIndex = i;
                }
                if(selectedIndex != -1 && targetArray[i].inRange){
                    if(fabs(targetArray[i].inLineY) < fabs(targetArray[i].inLineY)){
                        selectedIndex = i;
                    }
                }
            }
            if (selectedIndex == -1){
                return -1;
            } else {
                return targetArray[selectedIndex].requiredAngle;
            }
        }
};
/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Auton Methods               */
/*                Version 1.1.0               */
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
    mtrDriveLeftBack.startRotateTo(leftRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
    mtrDriveRightBack.startRotateTo(rightRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
}
void driveTurn(double degrees, double velocity){
    double currentHeading = (mtrDriveLeft.rotation(vex::rotationUnits::deg)-mtrDriveRight.rotation(vex::rotationUnits::deg)) * 90/684;
    double headingChange = degrees - currentHeading;
    double leftRotation = mtrDriveLeft.rotation(vex::rotationUnits::deg) + headingChange * 342/90;
    double rightRotation = mtrDriveRight.rotation(vex::rotationUnits::deg) - headingChange * 342/90;
    mtrDriveLeft.startRotateTo(leftRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
    mtrDriveRight.startRotateTo(rightRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
    mtrDriveLeftBack.startRotateTo(leftRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
    mtrDriveRightBack.startRotateTo(rightRotation, vex::rotationUnits::deg, velocity, vex::velocityUnits::pct);
}
/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Driver/Auton                */
/*                Version 1.1.0               */
/*--------------------------------------------*/
//Auton 1: Picks up ball, hits two flags, parks 
//Auton 2: Picks up ball, hits three flags
//Auton 3: Picks up ball, hits two flags, parks from back
//Auton 4: Picks up ball, hits two flags from back
//Auton 5: Skills: Picks up ball, hits three flags, center parks
void auton(int autonMode){
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(1,0);
    ctrPrimary.Screen.print("Autonomous");
    if (autonMode != 5){
        //Declare variable here
        int clock = 0;//Reset clock, motors, and process
        int process = 0; //variable to control where in the auton you are
        int nextProcess = 0;
        driveYawPID.reset();
        driveSpeedPID.reset();
        if (autonMode == 0){
            return;
        }
        while (confirmAuton() && process < 20){
            if ((process == -1 && nextProcess < 7) || (process < 7 && process != -1)){
                robot.liftBall(true, false, true);
            }
            if (process == 7){
                robot.liftBall(true, false, false);
            }
            switch (process){
                case -1://Case for pausing the auton until the drive motors are not moving, to allow for a timing system between processes
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        clock = 0;
                        process = nextProcess;
                    }
                    break;
                case 0://Drive towards ball under cap and pick it up
                    driveForward(1.75, 80);
                    process = -1;
                    nextProcess = 1;
                    break;
                case 1://Backup to firing position
                    if (clock >= 1000){
                        driveForward(-1.45, 60);
                        process = -1;
                        nextProcess = 2;
                    }
                    break;
                case 2://Drive towards the flags
                    if (clock >= 100){
                        if (autonMode == 1 || autonMode == 2 || autonMode == 5){
                            driveTurn((colorRed?-91:87), 60);
                        } else {
                            driveTurn((colorRed?-94:90), 60);
                        }
                        process ++;
                    }
                    break;
                case 3://lower the launcher down
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        robot.launchAngle(false, true);
                        process ++;
                        clock = 0;
                    }
                    break;
                case 4://stop the launcher and begin firing of both balls
                    if (clock >=500){
                        robot.launchAngle(false, false);
                        process ++;
                    }
                    break;
                case 7:
                    process ++;
                    break;
                case 10://Autons diverge here
                    if (autonMode == 1 || autonMode == 3){//Backup
                        driveForward(-0.8, 60);
                        process = -1;
                        nextProcess = 11;
                    }
                    if (autonMode == 2 || autonMode == 5){
                        process ++;
                    }
                    if (autonMode == 3){//Turn straight
                        driveTurn((colorRed?-90: 90), 60);
                        process = -1;
                        nextProcess = 11;
                    }
                    if (autonMode == 4){
                        process ++;
                    }
                    break;
                case 11: 
                    if (autonMode == 1){//Turn towards the platform
                        if (clock >= 100){
                            driveTurn((colorRed?-180:180), 60);
                            process = -1;
                            nextProcess = 12;
                        }
                    }
                    if (autonMode == 2 || autonMode == 5){
                        if (clock >= 100){
                            process ++;
                        }
                    }
                    if (autonMode == 3){//Drive Straight
                        if (clock >= 100){
                            driveForward(1.3, 60);
                            process = -1;
                            nextProcess = 12;
                        }
                    }
                    if (autonMode == 4){
                        process ++;
                    }
                    break;
                case 12:
                    if (autonMode == 1){//Drive onto platform
                        if(clock >= 100){
                            driveForward(-2, 100);
                            process = -1;
                            nextProcess = 13;
                        }
                    }
                    if (autonMode == 2 || autonMode == 5){//Turn to hit bottom flag
                        if(clock >= 100){
                            driveTurn((colorRed?-96:98), 60);
                            process = -1;
                            nextProcess = 13;
                        }
                    }
                    if (autonMode == 3){//Turn towards platform
                        if (clock >= 100){
                            driveTurn((colorRed?-180:180), 60);
                            process = -1;
                            nextProcess = 13;
                        }
                    }
                    if (autonMode == 4){
                        process ++;
                    }
                    break;
                case 13:
                    if (autonMode == 1 || autonMode == 4){//End of 1 and 4
                        process ++;
                    }
                    if (autonMode == 2 || autonMode == 5){//Drive forwards to hit the bottom flag
                        if (clock >= 100){
                            driveForward(1.9, 80);
                            process ++;
                        }
                    }
                    if (autonMode == 3){
                        if(clock >= 100){
                            driveForward(-2, 100);
                            process = -1;
                            nextProcess = 14;
                        }
                    }
                    break;
                case 14:
                    if (autonMode == 1 || autonMode == 3 || autonMode == 4){
                        process ++;
                    }
                    if (autonMode == 2 || autonMode == 5){//Drive backwards
                        if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                            driveForward(-1.9, 80);
                            process = -1;
                            nextProcess = 15;
                        }
                    }
                    break;
                case 15:
                    if (autonMode != 5){
                        process ++;
                    }
                    if (autonMode == 5){
                        if (clock >= 100){
                            driveTurn((colorRed?-90:90), 60);
                            process = -1;
                            nextProcess = 16;
                        }
                    }
                    break;
                case 16:
                    if (autonMode != 5){
                        process ++;
                    }
                    if (autonMode == 5){
                        if (clock >= 100){
                            driveForward(-1, 60);
                            process = -1;
                            nextProcess = 17;
                        }
                    }
                    break;
                case 17:
                    if (autonMode != 5){
                        process ++;
                    }
                    if (autonMode == 5){
                        if (clock >= 100){
                            driveTurn((colorRed?-180:180), 60);
                            process = -1;
                            nextProcess = 18;
                        }
                    }
                    break;
                case 18:
                    if (autonMode != 5){
                        process ++;
                    }
                    if (autonMode == 5){
                        if (clock >= 100){
                            driveForward(-3, 60);
                            process = -1;
                            nextProcess = 19;
                        }
                    }
                    break;
            }
            if (process == 5 || process == 6 || process == 8 || process == 9){//Line up and fire the ball launcher
                double angle = 0;
                if (autonMode == 1 || autonMode == 2 || autonMode == 5){
                    if (process == 5){
                        angle = 19;
                    }
                    if (process == 8){
                        angle = 33;
                    }
                }
                if (autonMode == 3 || autonMode == 4){
                    if (process == 5){
                        angle = 20;
                    }
                    if (process == 8){
                        angle = 27;
                    }
                }
                if (process == 5 || process == 8){
                    setLauncherToAngle(angle);
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
                if ((process == 5 || process == 8) && clock >= 1600){
                    mtrLauncherFire.startRotateTo(360 + mtrLauncherFire.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                    mtrLauncherFire2.startRotateTo(360 + mtrLauncherFire2.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);                    
                    process ++;
                    clock = 0;
                }
                if ((process == 6 || process == 9) && !mtrLauncherFire.isSpinning()){//Wait for the fire motor to stop
                    process ++;
                    clock = 0;
                }
            }
            wait(20);
            clock += 20;
        }
    }
    if (autonMode == 5){
        //Declare variable here
        int clock = 0;//Reset clock, motors, and process
        int process = 0; //variable to control where in the auton you are
        int nextProcess = 0;
        int liftProcess = 0;
        while (confirmAuton() && process < 52){
            switch (process){
                case -1:
                    if (!mtrDriveLeft.isSpinning() && !mtrDriveRight.isSpinning()){
                        clock = 0;
                        process = nextProcess;
                        break;
                    }
                    break;
                //retrieve ball
                case 0:
                    driveForward(1.75, 50);
                    process = -1;
                    nextProcess = 1;
                    liftProcess = 1;
                    break;
                //Flip cap
                case 1:
                    if (clock >= 2000){
                        driveForward(0.5, 30);
                        process = -1;
                        nextProcess = 2;
                        liftProcess = 2;
                    }
                    break;
                //Shoot 2 flags
                case 2: 
                    if (clock >= 100){
                        driveForward(-2.1, 80);
                        process = -1;
                        nextProcess = 3;
                        liftProcess = 1;
                    }
                    break;
                case 3:
                    if (clock >= 100){
                        driveTurn(-94, 60);
                        process = -1;
                        nextProcess = 4;
                    }
                    break;
                //Hit bottom flag, pick up balls on the way there
                case 8:
                    driveTurn(-90, 60);
                    process = -1;
                    nextProcess = 9;
                    liftProcess = 1;
                    break;
                case 9:
                    if (clock >= 100){
                        driveForward(2, 80);
                        process = -1;
                        nextProcess = 10;
                    }
                    break;
                case 10:
                    if (clock >= 100){
                        driveTurn(-98, 60);
                        process = -1;
                        nextProcess = 11;
                    }
                    break;
                case 11:
                    if (clock >= 100){
                        driveForward(2, 80);
                        process = -1;
                        nextProcess = 12;
                    }
                    break;
                case 12:
                    if (clock >= 100){
                        driveForward(-2, 80);
                        process = -1;
                        nextProcess = 13;
                        liftProcess = 3;
                    }
                    break;
                //Get ball
                case 13:
                    if (clock >= 100){
                        driveTurn(0, 60);
                        process = -1;
                        nextProcess = 14;
                    }
                    break;
                case 14:
                    if (clock >= 100){
                        driveForward(1.7, 50);
                        process = -1;
                        nextProcess = 15;
                        liftProcess = 1;
                    }
                    break;
                //Flip cap
                case 15:
                    if (clock >= 2000){
                        driveForward(0.5, 30);
                        process = -1;
                        nextProcess = 16;
                        liftProcess = 2;
                    }
                    break;
                //Turn and hit two flags
                case 16:
                    if (clock >= 100){
                        driveTurn(-95, 60);
                        process = -1;
                        nextProcess = 17;
                        liftProcess = 1;
                    }
                    break;
                //Hit bottom flag
                case 21:
                    driveTurn(-97, 60);
                    process = -1;
                    nextProcess = 22;
                    liftProcess = 2;
                    break;
                case 22:
                    if (clock >= 100){
                        driveForward(1.8, 80);
                        process = -1;
                        nextProcess = 23;
                    }
                    break;
                //flip cap
                case 23:
                    if (clock >= 100){
                        driveForward(-1.1, 80);
                        process = -1;
                        nextProcess = 24;
                    }
                    break;
                case 24:
                    if (clock >= 100){
                        driveTurn(0, 60);
                        process = -1;
                        nextProcess = 25;
                    }
                    break;
                case 25:
                    if (clock >= 100){
                        driveForward(2, 60);
                        process = -1;
                        nextProcess = 26;
                    }
                    break;
                //Turn and flip another cap
                case 26:
                    if (clock >= 100){
                        driveTurn(-180, 60);
                        process = -1;
                        nextProcess = 27;
                    }
                    break;
                case 27:
                    if (clock >= 100){
                        driveForward(3.5, 100);
                        process = -1;
                        nextProcess = 28;
                    }
                    break;
                //Park
                case 28:
                    driveTurn(-270, 40);
                    process = -1;
                    nextProcess = 29;
                    break;
                case 29:
                    if (clock >= 100){
                        driveForward(2,80);
                        process = -1;
                        nextProcess = 30;
                    }
                    break;
                case 30:
                    if (clock >= 100){
                        driveTurn(-180, 60);
                        process = -1;
                        nextProcess = 31;
                    }
                    break;
                case 31:
                    if (clock >= 100){
                        driveForward(-3.5, 100);
                        process = -1;
                        nextProcess = 32;
                        liftProcess = 0;
                    }
                    break;
            }
            switch (liftProcess){
                case 0:
                    robot.liftBall(false, false);
                    break;
                case 1:
                    robot.liftBall(true, false, true);//pull in and hold at limit switch
                    break;
                case 2:
                    robot.liftBall(false, true, true);//run backwards
                    break;
                case 3:
                    robot.liftBall(true, false, false);//pull in without limit
                    break;
            }
            //First fire sequence
            double angle = 0;
            if (process == 4){
                angle = 20;
            }
            if (process == 6){
                angle = 27;
            }
            if (process == 4 || process == 6){
                setLauncherToAngle(angle);
            }
            if (process == 5 || process == 7){
                mtrLauncherAngle.stop(vex::brakeType::hold);
            }
            if ((process == 4 || process == 6) && clock >= 1600){
                mtrLauncherFire.startRotateTo(360 + mtrLauncherFire.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                mtrLauncherFire2.startRotateTo(360 + mtrLauncherFire2.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                process ++;
                clock = 0;
            }
            if ((process == 5 || process == 7) && !mtrLauncherFire.isSpinning()){//Wait for the fire motor to stop
                process ++;
                liftProcess = 3;
                clock = 0;
            }
            //Second fire sequence
            if (process == 17){
                angle = 19;
            }
            if (process == 19){
                angle = 33;
            }
            if (process == 17 || process == 19){
                setLauncherToAngle(angle);
            }
            if (process == 18 || process == 20){
                mtrLauncherAngle.stop(vex::brakeType::hold);
            }
            if ((process == 17 || process == 19) && clock >= 1600){
                mtrLauncherFire.startRotateTo(360 + mtrLauncherFire.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                mtrLauncherFire2.startRotateTo(360 + mtrLauncherFire2.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
                process ++;
                clock = 0;
            }
            if ((process == 18 || process == 20) && !mtrLauncherFire.isSpinning()){//Wait for the fire motor to stop
                process ++;
                liftProcess = 3;
                clock = 0;
            }
            clock += 20;
            wait(20);
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
    bool waitForReleaseX = false;
    bool fire = true;
    mtrLauncherFire.startRotateTo(200, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
    mtrLauncherFire2.startRotateTo(200, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);

    while (confirmDriver()){
        //Run driver implementation here
        if (ctrPrimary.ButtonA.pressing() && !waitForReleaseA){//Turns on auto target
            waitForReleaseA = true;
            liftMode = !liftMode;
        }
        if (!ctrPrimary.ButtonA.pressing() && waitForReleaseA){//Waits for release to change the state of auto target
            waitForReleaseA = false;
        }
        if (ctrPrimary.ButtonX.pressing() && !waitForReleaseX){//Turns on auto target
            waitForReleaseX = true;
            fire = true;
            mtrLauncherFire.startRotateTo(360 + mtrLauncherFire.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
            mtrLauncherFire2.startRotateTo(360 + mtrLauncherFire2.rotation(vex::rotationUnits::deg), vex::rotationUnits::deg, 100, vex::velocityUnits::pct);
        }
        if (!ctrPrimary.ButtonX.pressing() && waitForReleaseX){//Waits for release to change the state of auto target
            waitForReleaseX = false;
        }
        if (ctrPrimary.ButtonLeft.pressing() || ctrPrimary.ButtonRight.pressing() || !fire){
            robot.launchFire(ctrPrimary.ButtonLeft.pressing(), ctrPrimary.ButtonRight.pressing());
            fire = false;
        }
        robot.launchAngle(ctrPrimary.ButtonR1.pressing(), ctrPrimary.ButtonR2.pressing());
        targetSystem.scanForFlags();//Run target system
        double angle = targetSystem.targetSpecificFlag();
        targetSystem.displayInformation();
        
        //Run the chassis
        int y = ctrPrimary.Axis3.position(vex::percentUnits::pct) * (liftMode?-1:1);//Run chassis with joystick, reverse the direction if the direction is reversed
        int x = ctrPrimary.Axis1.position(vex::percentUnits::pct);
        if (abs(ctrPrimary.Axis4.position(vex::percentUnits::pct)) > fabs((double)x)){
            x = ctrPrimary.Axis4.position(vex::percentUnits::pct);
        }
        robot.driveH(y, x);//Power the lift
        robot.liftBall(ctrPrimary.ButtonL1.pressing() || ctrSecond.ButtonL1.pressing(), ctrPrimary.ButtonL2.pressing() || ctrSecond.ButtonL2.pressing(), !ctrPrimary.ButtonUp.pressing() && !ctrSecond.ButtonUp.pressing());//Ball Lift
        runDiagnostics();//Check for warnings and display
        wait(50);//run at 50 Hz
    }
}
