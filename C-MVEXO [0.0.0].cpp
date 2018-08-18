/*--------------------------------------------*/
/*                    5249S                   */
/*             Communist ManiVEXo             */
/*--------------------------------------------*/

#include <cmath>

vex::motor mtrDriveLeft = vex::motor(vex::PORT1);
vex::motor mtrDriveRight = vex::motor(vex::PORT10);

//The methods used in these classes will not contain while loops, to prevent linearity and to prevent freezing of autons
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
    
        DriveMethods() { }
    
        Pid drive; //Creates a PID object for controlling driving
        Pid turn;  //Creates a PID object for controlling turning
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

            //motor[frontLeft] = leftMotor(a, t);
            //motor[backLeft] = leftMotor(b, t);
            //motor[frontRight] = rightMotor(b, t);
            //motor[backRight] = rightMotor(a, t);

        }
        void driveH(int y, int x){ //Method for driving the chassis with an h-drive, using turning
            //*Repeat method in while loop for continuous control, this is instant implementation for the motor power calculators*
            if (y < 10 && y > -10) {
                y = 0;
            }
            if (x < 10 && x > -10) {
                x = 0;
            }

            int leftPower = leftMotor(y, x);
            int rightPower = rightMotor(y, x);
            if (leftPower < 0){
                mtrDriveLeft.spin(rev, (double)(-leftPower), rpm);
            } else {
                mtrDriveLeft.spin(fwd, (double)leftPower, rpm);
            }
            
            if (rightPower < 0){
                mtrDriveRight.spin(rev, (double)(-rightPower), rpm);
            } else {
                mtrDriveRight.spin(fwd, (double)rightPower, rpm);
            }
            
        }
    
};
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
            if (abs(pidIntegral) < 50) {
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

void wait(int time){
    vex::timer timer;
    timer.clear();
    while (timer.time < time){}
}

int main() {
    vex::brain robotMain = vex::brain();
    vex::controller ctrPrimary = vex::controller();
    DriveMethods robot;
    controller.Screen.clearScreen();
    controller.Screen.setCursor(0, 0);
    controller.Screen.print("C-MVEXO 5249S");
    controller.Screen.newLine();
    controller.Screen.print("Ver: 0.0.0");
    controller.Screen.newLine();
    controller.Screen.print("Driver Control");

    while (true){
        int y = controller.Axis3.value();
        int x = controller.Axis1.value();

        robot.driveH(y, x);
        wait(20);
    }
    return 0;
}
