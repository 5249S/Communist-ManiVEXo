/*--------------------------------------------*/
/*                    5249S                   */
/*             Communist ManiVEXo             */
/*                Version 0.0.0               */
/*--------------------------------------------*/

#include <cmath>
#include "robot-config.h"

//The methods used in these classes will not contain while loops, to prevent linearity and to prevent freezing of autons
class Pid {
    private:
        //variables that affect pid that will remain hidden from user to increase stability/abstraction
        float prevError = 0;
        float pidIntegral = 0;
    public:
    
        Pid() { }
        
        float setPoint = 0;
        float kP = 0.0;
        float kI = 0.0;
        float kD = 0.0;
    
        float pidCalc(float processVar){
            //This is a single calculation for one cycle of a PID calculator, run in while loop, put output into control system
            //Define variables
            float pidProportional = 0.0;
            float pidDerivative = 0.0;
            float pidError = 0.0;

            //Calculate Error
            pidError = setPoint - processVar;

            //Calculate Proportional
            pidProportional = pidError * kP;
            
            //Calculate Derivative
            pidDerivative = (pidError - prevError) * kD;

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
            x *= -1;
            return leftMotor(y, x);
        }
    public:
    
        DriveMethods() { }
    
        Pid drive; //Creates a PID object for controlling driving
        Pid turn;  //Creates a PID object for controlling turning
        void driveH(int y, int x){ //Method for driving the chassis with an h-drive, using turning
            //*Repeat method in while loop for continuous control, this is instant implementation for the motor power calculators*
            if (y < 10 && y > -10) {//Threshold values to prevent drift
                y = 0;
            }
            if (x < 10 && x > -10) {
                x = 0;
            }

            int leftPower = leftMotor(y, x);//Gets the power for the left and right side of the chassis
            int rightPower = rightMotor(y, x);
            if (leftPower < 0){//Powers the motors to the correct power
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


void wait(int time){//function to wait a specific amount of milliseconds
    vex::timer timer;
    timer.clear();
    while (timer.time() < time){}
}

int main() {
    DriveMethods robot;
    ctrPrimary.Screen.clearScreen();//Displays name and version number on controller screen
    ctrPrimary.Screen.setCursor(0, 0);
    ctrPrimary.Screen.print("C-MVEXO 5249S");
    ctrPrimary.Screen.newLine();
    ctrPrimary.Screen.print("Ver: 0.0.0");
    ctrPrimary.Screen.newLine();
    ctrPrimary.Screen.print("Driver Control");

    while (true){
        int y = ctrPrimary.Axis3.position(vex::percentUnits::pct);//Runs robot chassis based on joystick inputs
        int x = ctrPrimary.Axis1.position(vex::percentUnits::pct);

        robot.driveH(y, x);
        wait(20);
    }
    return 0;
}
