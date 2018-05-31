/*--------------------------------------------*/
/*                    5249S                   */
/*             Communist ManiVEXo             */
/*--------------------------------------------*/

#include <iostream>
#include <cmath>
using namespace std;

class DriveMethods {
    //Class for methods for driving the robot around the field
    protected:
        //Calculators for motor power to translate remote joystick values into motor powers
        int leftMotor(int y, int x){
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
        void driveX(int y, int x, int t){ //Method for driving the chassis with an x-drive, incorporating strafe and turning
            //*Repeat method in while loop for continuous control, this is instant implementation for the motor power calculators*
            if (y < 10 && y > -10) {
                y = 0;
            }
            if (x < 10 && x > -10) {
                x = 0;
            }
            if(t < 10 && t > -10){
                t = 0;
            }

            int a = 0;
            int b = 0;

            a = (int) (-x+y)*(sqrt(2)/2);
            b = (int) (x-y)*(sqrt(2)/2);

            motor[frontLeft] = leftMotor(a, t);
            motor[backLeft] = leftMotor(b, t);
            motor[frontRight] = rightMotor(b, t);
            motor[backRight] = rightMotor(a, t);

        }
        void driveH(int y, int x){ //Method for driving the chassis with an h-drive, using turning
            //*Repeat method in while loop for continuous control, this is instant implementation for the motor power calculators*
            if (y < 10 && y > -10) {
                y = 0;
            }
            if (x < 10 && x > -10) {
                x = 0;
            }

            motor[frontLeft] = leftMotor(y, x);
            motor[backLeft] = leftMotor(y, x);
            motor[frontRight] = rightMotor(y, x);
            motor[backRight] = rightMotor(y, x);
        }
    
}
class Launcher { //Class for measuring and calculating whether the robot's ball launcher in it's current state will hit a flag
    private: 
        //All measurements use SI units
        const float GRAVITY = 9.8; //meters per second^2
        const float INITIAL_VELOCITY = 0; //To be calculated
        const float FLAG_HEIGHT = 0; //Centimeters
        const int BALL_MASS = 55; //mass in grams
        const float k = 0; //Constant of drag
        float distance = 0;
        float height = 0;
        float pi = 3.1415;
        float e = 2.7182;
        senseVision(){//Uses vision sensor to calculate horizontal distance away from the flag and height from 'horizon' to bottom of flag
            float alpha;
            float beta;
            
            distance = FLAG_HEIGHT/(tan(beta) - tan(alpha);//Uses trig to determine values an changes instance variables to them
            height = distance * tan(alpha);
        }
        
    public:
        ballToFlagCheck(float x = distance){//Using kinematic formulas, use the launcher angle to determine if the ball will hit the flag
            float a = INITIAL_VELOCITY * cos(gyroLauncher.angle() * pi / 180);//Calculate horizontal velocity component
            float b = INITIAL_VELOCITY * sin(gyroLauncher.angle() * pi / 180);//Calculate verticle velocity component
            
            float y = -(GRAVITY * x^2)/(2 * a^2) + b * x/a; //Calculate what the height will be at the given distance
            if(y >= height + 0.5 && y >= height + FLAG_HEIGHT - 0.5){//Determine if that height is at the flag's height
                return true;
            }
            return false;
        }
        ballToFlagCheckDrag(float x = distance){//Using kinematic formulas and using an equation for drag, F= -k*v, use the launcher angle to determine if the ball will hit the flag
            float a = INITIAL_VELOCITY * cos(gyroLauncher.angle() * pi / 180);//Calculate horizontal velocity component
            float b = INITIAL_VELOCITY * sin(gyroLauncher.angle() * pi / 180);//Calculate verticle velocity component
            if(k == 0){//If constant of drag is 0, return regular kinematic formula answer to prevent math error
                return ballToFlagCheck();
            }
            float mass = BALL_MASS / 1000;//convert mass to kilograms
            float t = (BALL_MASS/k) * log(1 - x/((BALL_MASS/k)*a)); //Calculate time until distance reached
            float y = -(GRAVITY * mass / k)*t + (mass/ k)*(b + (GRAVITY * mass/k))*(1 - e^(-k*t/ mass));//Calculate height at that time
            if(y >= height + 0.5 && y >= height + FLAG_HEIGHT - 0.5){//Determine if that height is at the flag's height
                return true;
            }
            return false;
        }
        updateRemoteControlInterface(){}
    
}
class Pid: public DriveMethods {
    public:
        struct pidCalcReturn {
            float adjust;
            float lastError;
            float integral;
        }
        pidCalcReturn pidCalc(float processVar, float setPoint = 0, float prevError = 0, float pidIntegral = 0, float kP = 0.0, float kI = 0.0, float kD = 0.0){
            //This is a single calculation for one cycle of a PID calculator, the integral and previous error variables need to be cycled back into the calculator
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
            if (abs(pidIntegral) < 50) {
                    pidIntegral = pidError - (pidError-prevError)/2 + pidIntegral;
            } else {
                    pidIntegral = 0;
            }

            //use pid structure to return values
            pidCalcReturn returnValues;
            returnValues.adjust = pidDerivative + pidProportional + kI * pidIntegral;
            returnValues.lastError = pidError;
            returnValues.integral = pidIntegral;

            //return adjustment, new error, and pidIntegral
            return returnValues;
        }
}
}
class RoboMethods: public Pid {

}
