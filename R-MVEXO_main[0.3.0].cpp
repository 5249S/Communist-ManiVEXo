/*--------------------------------------------*/
/*                    5249S                   */
/*                The ManiVEXo                */
/*                    Main                    */
/*                Version 0.3.0               */
/*--------------------------------------------*/
#include "robot-config.h"
#include <cmath>
    
static int mode = -1;
static bool colorRed = true;
void auton(int);
void driver();
static bool warning[10][2];
double getAccelTiltAngle(){
    double calibrationParam[4][3] = {{-328141E-10,36174E-8,736303E-10},{398013E-10,-469610E-10,-372480E-8},{-350956E-8,-130104E-10,-459309E-10},{7.20023,-7.29033,7.57130}};
    int X = accelLauncherX.value(vex::analogUnits::range12bit);
    int Y = accelLauncherY.value(vex::analogUnits::range12bit);
    int Z = accelLauncherZ.value(vex::analogUnits::range12bit);
    double measuredValues[4] = {(double)X,(double)Y,(double)Z,1};
    double trueValues[3];
    for (int i = 0;i < 3; i++){
        double dotSum = 0;
        for (int j = 0; j < 4; j++){
            dotSum += measuredValues[j]*calibrationParam[j][i];
        }
        trueValues[i] = dotSum;
    }
    return (180/3.141592)*atan((trueValues[0])/(sqrt(pow(trueValues[1],2)+pow(trueValues[2],2))));
}
void runDiagnostics(){//Method for displaying any problems with the robot
    char warningText[10][6] = {"BatL ","BatH ","MdlH ","MdrH ","MllH ","MlrH","","","",""};//array of warning texts
    for (int i = 0; i < 10; i++){ //store the previous state of each error to check for a change
        warning[i][1] = warning[i][0];
    }
    warning[0][0] = robotMain.Battery.capacity() < 25;//Battery capacity < 25%
    warning[1][0] = robotMain.Battery.temperature() > 80; //Battery temperature > 80%
    warning[2][0] = mtrDriveLeft.temperature(vex::percentUnits::pct) > 70; //Left Drive Motor >70%
    warning[3][0] = mtrDriveRight.temperature(vex::percentUnits::pct) > 70; //Right Drive Motor >70%
    warning[4][0] = mtrLiftLeft.temperature(vex::percentUnits::pct) > 70; //Left Lift Motor >70%
    warning[5][0] = mtrLiftRight.temperature(vex::percentUnits::pct) > 70; //Right Lift Motor >70%
    warning[6][0] = false;
    warning[7][0] = false;
    warning[8][0] = false;
    warning[9][0] = false;
    
    
    bool update = false;
    for (int i = 0; i < 10; i++){
        if ((warning[i][0] && !warning[i][1]) || (warning[i][1] && !warning[i][0])){ //Update the display if any of the warnings have changed
            update = true; 
            break;
        }
    }
    if (update) {//Display all warnings
        ctrPrimary.Screen.clearLine(2);
        ctrPrimary.Screen.clearLine(3);
        ctrPrimary.Screen.setCursor(2,0);
        for (int i = 0; i < 10; i++){
            if (warning[i][0]){
                ctrPrimary.Screen.print("%s ", warningText[i]);
            }
        }
    }
}

class GyroSettings {//Class used to set gyros to specific values, as they can't be changed in the program
    private:
        int gyroBias = 0;
        int reverse = 1;
    public:
        void setValues(int trueValue, int currentValue, bool rev){//Sets proper values
            reverse = rev?-1:1;
            gyroBias = currentValue - reverse * trueValue;
        }
        int value(int currentValue){//returns true value with wanted shift
            return reverse * (currentValue - gyroBias);
        }
};
void wait(int time){
    vex::task::sleep(time);
}
GyroSettings gyroLauncherSet;
GyroSettings gyroNavSet;
void calibrateGyros(){//Calibrates gyros
    robotMain.Screen.clearScreen();
    robotMain.Screen.setCursor(0,0);
    robotMain.Screen.print("Gyros Calibrating");
    robotMain.Screen.newLine();
    robotMain.Screen.print("Do Not Touch Robot!");
    robotMain.Screen.newLine();
    robotMain.Screen.print("(B) Bypass");
    gyroNav.startCalibration();
    gyroLauncher.startCalibration();
    int timer = 0;
    while((gyroNav.isCalibrating() || gyroLauncher.isCalibrating()) && timer < 3000){//waits for both gyros to finish 
        if (ctrPrimary.ButtonB.pressing()){
            break;//allows bypass
        }
        wait(20);
        timer += 20;
    }
    wait(3000);
    while(ctrPrimary.ButtonB.pressing()){wait(20);}
    gyroNavSet.setValues(0, gyroNav.value(vex::rotationUnits::deg), false);
    gyroLauncherSet.setValues(0, gyroLauncher.value(vex::rotationUnits::deg), false);
    robotMain.Screen.clearScreen();
    
    
}
void stopAllMotors(){//stops all motors on the robot
    mtrDriveLeft.stop(vex::brakeType::coast);
    mtrDriveRight.stop(vex::brakeType::coast);
    mtrLiftLeft.stop(vex::brakeType::coast);
    mtrLiftRight.stop(vex::brakeType::coast);
    mtrClaw.stop(vex::brakeType::coast);
    mtrLauncherAngle.stop(vex::brakeType::coast);
    mtrLauncherFire.stop(vex::brakeType::coast);
}
bool isField(){//Method for checking if either field control device is connected
    return compControl.isCompetitionSwitch() || compControl.isFieldControl();
}
class DisplaySelection {//Class created to hold and change the values needed to move the display up and down
        private: 
            int maxLines = 3;
            int topLine = 0;
            int position = 0;
            unsigned int max = 0;
            bool selectionMade = false;
    
            int getCurrent(){
                return topLine + position;
            }
            void moveDown(){
                if (getCurrent() != max - 1){
                    if (position == maxLines - 1){
                        topLine ++;
                    } else {
                        position ++;
                    }
                } else {
                    topLine = 0;
                    position = 0;
                }
            }
            void moveUp(){
                if (getCurrent() != 0){
                    if (position == 0){
                        topLine --;
                    } else {
                        position --;
                    }
                } else {
                    position = maxLines - 1;
                    topLine = max - maxLines;
                }
            }
        public:
            char text[8][32];
            DisplaySelection(unsigned int maxOptions){
                if (maxOptions < maxLines){
                    maxLines = maxOptions;
                }
                max = maxOptions;
            }
            int select(){
                while(true){//repeat update until a selection is chosen
                    if(ctrPrimary.ButtonA.pressing()){//Return the current number if a selection has been made
                        return getCurrent();
                    }
                    if(ctrPrimary.ButtonUp.pressing()){
                        moveUp();
                    } 
                    if(ctrPrimary.ButtonDown.pressing()){
                        moveDown();
                    }
                    ctrPrimary.Screen.clearScreen();
                    for (int i=0; i < maxLines; i++){
                        ctrPrimary.Screen.setCursor(i+1,3);
                        ctrPrimary.Screen.print("%s", text[i + topLine]);
                    }
                    ctrPrimary.Screen.setCursor(position+1,0);
                    ctrPrimary.Screen.print("->");
                    while(ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonUp.pressing() || ctrPrimary.ButtonDown.pressing()){wait(20);}//wait for all buttons to be released
                    while(!(ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonUp.pressing() || ctrPrimary.ButtonDown.pressing())){
                        if (isField()){
                            ctrPrimary.Screen.clearScreen();
                            ctrPrimary.Screen.setCursor(1,0);
                            ctrPrimary.Screen.print("Remove Field Cable");
                            while (isField()){
                                wait(20);
                            }
                            break;
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
    DisplaySelection selectAuton = DisplaySelection(4);//create display selection object
    strcpy(selectAuton.text[0], "Exit");//place names of autons in array
    strcpy(selectAuton.text[1], "Auton1");
    strcpy(selectAuton.text[2], "Auton2");
    strcpy(selectAuton.text[3], "Auton3");
    strcpy(selectAuton.text[4], "");
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
            colorSelect();//select team color
            int autonMode = selectAutonomous();//select auton to run
            while(true){//loop for competition
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
                while(!compControl.isEnabled()){//While disabled, user has option to close field control 
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.clearLine();
                    ctrPrimary.Screen.print("FC-Disabled");
                    while(!compControl.isEnabled()){wait(20);}
                }
                
                if(compControl.isEnabled() && compControl.isAutonomous()){//runs auton when enabled and autonomous
                        auton(autonMode);
                        while(compControl.isEnabled() && compControl.isAutonomous() && isField()){wait(20);}//Waits for auton to end (50 Hertz)
                }
                if(compControl.isEnabled() && compControl.isDriverControl()){//runs driver control when enabled and autonomous
                    driver();
                    while(compControl.isEnabled() && compControl.isDriverControl() && isField()){wait(20);}//Waits for driver control to end (50 Hertz)
                }
                stopAllMotors();
            }
            stopAllMotors();
        }
        if(mode == 1){//Skills mode- same as field control, except no color selection
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
                while(!compControl.isEnabled()){//While disabled, user has option to close field control 
                    ctrPrimary.Screen.setCursor(1,0);
                    ctrPrimary.Screen.clearLine();
                    ctrPrimary.Screen.print("SK-Disabled");
                    while(!compControl.isEnabled()){wait(20);}
                }
                if(compControl.isEnabled() && compControl.isAutonomous()){
                        auton(autonMode);
                        while(compControl.isEnabled() && compControl.isAutonomous() && isField()){wait(20);}//Waits for auton to end (50 Hertz)
                }
                if(compControl.isEnabled() && compControl.isDriverControl()){
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
                    if (ctrPrimary.ButtonA.pressing()){
                        break;
                    }
                    wait(50); //Update at 20 hertz
                }
                while(ctrPrimary.ButtonA.pressing()){wait(20);}
                calibrateGyros();
                int selection = selectAutonomous();
                if(selection == 0){
                    break;
                }
                colorSelect();//select color
                auton(selection);//run selected auton
                stopAllMotors();
                while(ctrPrimary.ButtonB.pressing()){wait(20);}//wait for exit button to be released
            }
        }
        if(mode == 3){
            calibrateGyros();
            driver();
            stopAllMotors();
            while(ctrPrimary.ButtonB.pressing()){wait(20);}//wait for exit button to be released
        }
        if(mode == 4){
            return 0;
        }
    }
}
