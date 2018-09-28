/*--------------------------------------------*/
/*                    5249S                   */
/*             Communist ManiVEXo             */
/*                   Methods                  */
/*                Version 0.1.0               */
/*--------------------------------------------*/

class GyroSettings {
    private:
        gryoBias = 0;
        reverse = 1;
    public:
        void setValues(int trueValue, int currentValue, bool rev){
            reverse = rev?-1:1;
            gyroBias = currentValue - reverse * trueValue;
        }
        int value(int currentValue){
            return reverse * (currentValue - gyroBias);
        }
};
void wait(int time){
    vex::task::sleep(time);
}
GyroSettings gyroLauncherSet;
GyroSettings gyroNavSet;
void calibrateGyros(){
    ctrPrimary.Screen.clearScreen();
    ctrPrimary.Screen.setCursor(0,0);
    ctrPrimary.Screen.print("Gyros Calibrating")
    ctrPrimary.Screen.newLine();
    ctrPrimary.Screen.print("Do Not Touch Robot");
    ctrPrimary.Screen.newLine();
    ctrPrimary.Screen.print("(B) Bypass");
    gyroNav.startCallibration();
    gyroLauncher.startCallibration();
    while(gryoNav.isCalibrating() || gryoLauncher.isCalibrating()){
        if (ctrPrimary.ButtonB.pressing()){
            break;
        }
        wait(20);
    }
    while(ctrPrimary.ButtonB.pressing()){wait(20);}
    gyroLauncherSet.setValues(gyroLauncher.value(vex::rotationUnits::deg, false);
    
    
}
void stopAllMotors(){

}

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
