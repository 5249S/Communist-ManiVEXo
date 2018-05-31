/*--------------------------------------------*/
/*                    5249S                   */
/*             Communist ManiVEXo             */
/*--------------------------------------------*/

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
