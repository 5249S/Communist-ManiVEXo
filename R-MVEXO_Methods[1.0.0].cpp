/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                   Methods                  */
/*                Version 0.3.0               */
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
            if (up && getAccelTiltAngle() < 32){
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
                if (power > 0 && getAccelTiltAngle() < 32){
                    mtrLauncherAngle.spin(vex::directionType::fwd, (double)power, vex::velocityUnits::pct);
                } else {
                    mtrLauncherAngle.stop(vex::brakeType::hold);
                }
            }
        }
        void launchFire(bool power){//Fire the ball launcher
            if (power){
                mtrLauncherFire.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
            } else {
                mtrLauncherFire.stop(vex::brakeType::coast);
            }
        }
};
class Claw {//Methods for running the claw
    public: 
        void claw(bool up, bool down){//Run with booleans
            if (up){
                mtrClaw.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
            } else {
                if(down){
                    mtrClaw.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
                } else {
                    mtrClaw.stop(vex::brakeType::hold);
                }
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
class Navigation {
    private:
        double positionX = 0;
        double positionY = 0;
        double velocityX = 0;
        double velocityY = 0;
    
        double prevAccelX = 0;
        double prevAccelY = 0;
        double prevVelocityX = 0;
        double prevVelocityY = 0;
        const double toRad = 3.14159265/1800;
        double getAccelXValue(){
            double accelOffsetX = 0;
            double analogPerGX = 0;
            return ((double)accelNavX.value() - accelOffsetX)/analogPerGX * 9.8;
        }
        double getAccelYValue(){
            double accelOffsetY = 0;
            double analogPerGY = 0;
            return ((double)accelNavY.value() - accelOffsetY)/analogPerGY * 9.8;
        }
    public:
        double getXPosition(){
            return positionX;
        }
        double getYPosition(){
            return positionY;
        }
        void calculatePositions(int time){
            accelX = getAccelXValue();
            accelY = getAccelYValue();
            velocityX += (accelX + prevAccelX) * (time/1000) / 2;
            velocityY += (accelY + prevAccelY) * (time/1000) / 2;
            positionX += (velocityX + prevVelocityX) * (time/1000) / 2;
            positionY += (velocityY + prevVelocityY) * (time/1000) / 2;
            
            prevAccelX = accelX;
            prevAccelY = accelY;
            prevVelocityX = velocityX;
            prevVelocityY = velocityY;
        }
}
class RobotControl: public Lift, public DriveMethods, public Claw, public Launcher, public BallLift {//Combine methods into one class
    
};
