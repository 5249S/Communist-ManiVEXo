//Not ready to be used on robot, needs to be refined
class Flag {
    private:
        const double GRAVITY = -9.8; //m/s^2 Acceleration of gravity
        const double INITIAL_VELOCITY = 0; //m/s Initial velocity of ball
        const double FLAG_HEIGHT = 0; //meters known height of flag object
        float double = 0; //Variable for holding distance
        float double = 0; //Variable holding height to bottom of flag
        
    public:
        //Equations are based on inmatic formulas
        void calculateDistance(double alpha, double beta){//Takes angle values and sets instance varibles to distance an height
            int offset = 0; //
            distance = (float)(FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (float)(tan(alpha));
        }
        float calculateRequiredAngle(){//Returns angle required to hit flag in radians
            double v = INITIAL_VELOCITY;//variables to hold values, for simplicity
            double g = GRAVITY;
            double h = height + FLAG_HEIGHT/2;//height of middle of flag
            double d = distance;
            int offset = 0.0;//Offset varible to adjust for systematic error
            return (float)(atan((pow(v,2.0) - sqrt(pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0))))/(g*d) + offset));//calculates angle required, casting between numbers where needed
        }
        bool checkForHit(){
            return false;
        }
};
class Launcher : private Pid {
    private:
        
        int htzIndex = 0;//Number of flags in horizontal target zone
        const double pi = 3.141592;//Pi
        double toRad(double degrees){//Converts degrees to radians
            return degrees * (pi/180.0);
        }
        double toDeg(double radians){//converts radians to degrees
            return radians * (180.0/pi);
        }
        int accelToGyro(){//Sets gyro angle to 3- axis accelerometer based on gravity
            int Z = accelLauncherZ.value(vex::analogUnits::range12bit);//Stores accelerometer values
            int Y = accelLauncherY.value(vex::analogUnits::range12bit);
            int offset = 0;//Offset varible to adjust for systematic error
            if (Y == 0){//Angle is either 0 or 90 degrees
                if (Z == 0){
                    return 0;//Something is wrong if this happens, only here to prevent error
                }
                return (int)((toDeg(-asin((double)Z/abs(Z)))) + offset);//gives back either 0 or 90 degrees
            }
            return (int)(toDeg(-atan((double)(Z/Y))) + offset);//Calculates angle
        }
        Flag htzFlags[9];
        //The following values are for the vision sensor camera
        const float FOV = 47.0;//field of view
        const float FOCAL_LENGTH = 200/tan(toRad(FOV/2));//Focal length of the camera based on field of view 
        const int htzMax = 340;//Upper limit of horizontal target zone
        const int htzMin = 300;//Lower limit of horizontal target zone
        float angleAtPoint(int y){//calculates the verticle angle to a specific point on the camera using the field of view and focal length
            float offset = 0;//Offset for systematic error
            int yP = y - 200;//Makes center the origin
            return (float)(toDeg(atan((double)(yP/FOCAL_LENGTH)))) + offset;//Returns angle at point
        }
        
        void runAngleMotor(int angle){//Runs motor to set launcher to an angle
            
            setPoint = angle;
            float fix = pidCalc(gyroLauncherSet.value(gyroLauncher.value(vex::rotationUnits::deg)));
            if (fix > 100){
                return;
            }
            if (fix < -100){
                return;
            }
        }
    public:
        Launcher(){//Constructor, just sets gyro to 
            gyroLauncherSet.setValues(accelToGyro(), gyroLauncher.value(vex::rotationUnits::deg), false);
            kP = 0.0;
            kI = 0.0;
            kD = 0.0;
        }
        int flagX[9];
        void scanForFlags(){//
            if (colorRed){
                visLauncher.takeSnapshot(SIG_BLUE_FLAG);
            } else {
                visLauncher.takeSnapshot(SIG_RED_FLAG);
            }
            htzIndex = 0;
            for (int i = 0; i < visLauncher.objectCount; i++){
                if (visLauncher.objects[i].centerX < htzMax && visLauncher.objects[i].centerX > htzMin){
                    float beta = angleAtPoint(visLauncher.objects[i].originY);
                    float alpha = angleAtPoint(visLauncher.objects[i].originY - 2 * (visLauncher.objects[i].originY - visLauncher.objects[i].centerY));
                    htzFlags[htzIndex].calculateDistance(alpha, beta);
                    htzIndex ++;
                }
                flagX[i] = visLauncher.objects[i].centerX;
            }
        }
        void targetSpecificFlag(){
            float angles[htzIndex];
            for (int i = 0; i < htzIndex; i++){//Calculate the difference between the needed angle and gyro angle for all flags in the htz
                angles[i] = fabs(htzFlags[i].calculateRequiredAngle() - (float)gyroLauncherSet.value(gyroLauncher.value(vex::rotationUnits::deg)));
            }
            int shortestAngle = 0;//set the first angle to the shortest angle
            for (int i = 1; i < htzIndex; i++){
                if (angles[i] < angles[shortestAngle]){//If any other angle is closer to the gyro angle, set that to the closest angle
                    shortestAngle = i;
                }
            }
            runAngleMotor((int)htzFlags[shortestAngle].calculateRequiredAngle());//Run the motor to reach selected angle.
        }
};
