/*--------------------------------------------*/
/*                    5249S                   */
/*                The ManiVEXo                */
/*                    Main                    */
/*                Version 0.1.0               */
/*--------------------------------------------*/
#include "robot-config.h"
#include <cmath>
    
static int mode = -1;
static bool colorRed = true;
void auton(int);
void driver();
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
    robotMain.Screen.print("Do Not Touch Robot");
    robotMain.Screen.newLine();
    robotMain.Screen.print("(B) Bypass");
    gyroNav.startCalibration();
    gyroLauncher.startCalibration();
    while(gyroNav.isCalibrating() || gyroLauncher.isCalibrating()){//waits for both gyros to finish 
        if (ctrPrimary.ButtonB.pressing()){
            break;//allows bypass
        }
        wait(20);
    }
    while(ctrPrimary.ButtonB.pressing()){wait(20);}
    gyroNavSet.setValues(0, gyroNav.value(vex::rotationUnits::deg), false);
    gyroLauncherSet.setValues(0, gyroLauncher.value(vex::rotationUnits::deg), false);
    
    
}
void stopAllMotors(){
    mtrDriveLeft.stop(vex::brakeType::coast);
    mtrDriveRight.stop(vex::brakeType::coast);
    mtrLiftLeft.stop(vex::brakeType::coast);
    mtrLiftRight.stop(vex::brakeType::coast);
    
}     
class DisplaySelection {//Class created to hold and change the values needed to move the display up and down
        private:
            int maxLines = 5;
            int current = 0;
            int topLine = 0;
            bool selectionMade = false;

            unsigned int max = 0;
            int getPosition(){
                return current - topLine;
            }

            void moveDown(){
                if (current != max - 1){
                    if (current == topLine + maxLines - 1){
                        topLine ++;
                    }
                    current ++;
                }
            }
            void moveUp(){
                if (current != 0){
                    if (current == topLine){
                        topLine --;
                    }
                    current --;
                }
            }
            int update(bool select, bool up, bool down){
                if(select){
                    return current;
                }
                if(up){
                    moveUp();
                } 
                if(down){
                    moveDown();
                }
                robotMain.Screen.clearScreen();
                for (int i=0; i < maxLines; i++){
                    robotMain.Screen.setCursor(i+1,3);
                    robotMain.Screen.print("%s", text[i + topLine]);
                }
                robotMain.Screen.setCursor(getPosition()+1,0);
                robotMain.Screen.print("->");
                return -1;
            }
        public:
            char text[8][32];
            DisplaySelection(unsigned int maxOptions){
                max = maxOptions;
            }
            int select(){
                while(true){//repeat update until a selection is chosen
                    bool selectBtn = ctrPrimary.ButtonA.pressing();
                    bool up = ctrPrimary.ButtonUp.pressing();
                    bool down = ctrPrimary.ButtonDown.pressing();
                    int status = update(selectBtn, up, down);//call update function
                    while(ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonUp.pressing() || ctrPrimary.ButtonDown.pressing()){wait(20);}//wait for all buttons to be released
                    if (status != -1){//repeat loop until selection is made (update return something other than -1)
                        return status;//return number selected
                        break;
                    }
                    wait(50); //Update at 20 hertz
                }
            }

};
class PromptClose {//Handles whether the user wants to exit at a particular screen
    private:
        bool prompt = false;//bool for whether the second confirm close screen should be shown
    public:
        int update(bool A, bool B){//A and B are the values for selecting the different options
            robotMain.Screen.setCursor(1,0);
            robotMain.Screen.clearLine();
            robotMain.Screen.setCursor(2,0);
            robotMain.Screen.clearLine();
            if((!prompt && B) || (prompt && !A && !B)){//Shows second screen if prompt is true or b is chosen on first screen
                prompt = true;
                robotMain.Screen.setCursor(1,0);
                robotMain.Screen.print("(A) Close?");
                robotMain.Screen.newLine();
                robotMain.Screen.print("(B) Back");
                return 0;
            }
            if((prompt && B) || (!prompt && !B)){//Shows first screen if prompt is false or if b is chosen on the second screen
                prompt = false;
                robotMain.Screen.setCursor(1,0);
                robotMain.Screen.print("(B) Close");
                return 0;
            }
            return 1;//returns 0 if there is no close chosen, returns 1 if close is chosen
        }
};
bool confirmAuton(){//Confirms it is allowed to run auton
    if (mode == 0 || mode == 1){//If in field control or skills mode, the competition control will be checked
        if (compControl.isAutonomous() && compControl.isEnabled()){//return true if auton is on and the robot is enabled
            return true;
        }
        return false;//otherwise return false
    }
    if (mode == 2){//if in auton testing mode, always allow
        return true;
    }
    return false;//return false otherwise
}
bool confirmDriver(){//Confirms it is allowed to run driver control
    if (mode == 0 || mode == 3){//If in field control or skills mode, the competition control will be checked
        if (compControl.isDriverControl() && compControl.isEnabled()){//return true if driver is on and the robot is enabled
            return true;
        }
        return false;//otherwise return false
    }
    if (mode == 2){//if in driver mode, always allow
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
    return selectAuton.select();
}
void colorSelect(){//method for selecting field color
    DisplaySelection selectColor = DisplaySelection(2);//create display object
    strcpy(selectColor.text[0], "Red");//set array values to colors
    strcpy(selectColor.text[1], "Blue");
    colorRed = (selectColor.select() == 0);
}
int main() {
    robotMain.Screen.setFont(vex::fontType::mono40);
    robotMain.Screen.clearScreen();
    robotMain.Screen.setCursor(1,0);
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
            robotMain.Screen.clearScreen();
            while(true){//loop for competition
                PromptClose promptExit = PromptClose();
                bool statusClose = false;
                while(!compControl.isEnabled()){//While disabled, user has option to close field control 
                    robotMain.Screen.setCursor(1,0);
                    robotMain.Screen.clearLine();
                    robotMain.Screen.print("FC-Disabled");
                    bool a = ctrPrimary.ButtonA.pressing();
                    bool b = ctrPrimary.ButtonB.pressing();
                    statusClose = (promptExit.update(a, b) == 1);
                    while((ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonB.pressing()) && !compControl.isEnabled()){wait(20);}
                    if (statusClose){//allow robot to exit competition when disabled
                        break;
                    }
                    wait(20);
                }
                if (statusClose){
                    break;
                }
                if(compControl.isEnabled() && compControl.isAutonomous()){//runs auton when enabled and autonomous
                        auton(autonMode);
                        while(compControl.isEnabled() && compControl.isAutonomous()){wait(20);}//Waits for auton to end (50 Hertz)
                }
                if(compControl.isEnabled() && compControl.isDriverControl()){//runs driver control when enabled and autonomous
                    driver();
                    while(compControl.isEnabled() && compControl.isDriverControl()){wait(20);}//Waits for driver control to end (50 Hertz)
                }
                stopAllMotors();
            }
            stopAllMotors();
        }
        if(mode == 1){//Skills mode- same as field control, except no color selection
            colorRed = true;
            int autonMode = selectAutonomous();
            robotMain.Screen.clearScreen();
            while(true){
                bool statusClose = false;
                PromptClose promptExit = PromptClose();
                while(!compControl.isEnabled()){
                    robotMain.Screen.setCursor(1,0);
                    robotMain.Screen.clearLine();
                    robotMain.Screen.print("SK-Disabled");
                    bool a = ctrPrimary.ButtonA.pressing();
                    bool b = ctrPrimary.ButtonB.pressing();
                    statusClose = (promptExit.update(a, b) == 1);
                    while((ctrPrimary.ButtonA.pressing() || ctrPrimary.ButtonB.pressing()) && !compControl.isEnabled()){wait(20);}
                    if (statusClose){
                        break;
                    }
                    wait(20);
                }
                if (statusClose){
                    break;
                }
                if(compControl.isEnabled() && compControl.isAutonomous()){
                        auton(autonMode);
                        while(compControl.isEnabled() && compControl.isAutonomous()){wait(20);}//Waits for auton to end (50 Hertz)
                }
                if(compControl.isEnabled() && compControl.isDriverControl()){
                    driver();
                    while(compControl.isEnabled() && compControl.isDriverControl()){wait(20);}//Waits for driver control to end (50 Hertz)
                }
                stopAllMotors();
            }
            stopAllMotors();
        }
        if(mode == 2){//Auton testing mode
            while (true){
                while(true){
                    robotMain.Screen.clearScreen();
                    robotMain.Screen.setCursor(1,0);
                    robotMain.Screen.print("Setup Robot");
                    robotMain.Screen.newLine();
                    robotMain.Screen.print("(A) Done");//Waits until robot is placed properly for auton
                    if (ctrPrimary.ButtonA.pressing()){
                        break;
                    }
                    wait(20);//50 hertz
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
