class Flag {
    private:
        const double GRAVITY = -9.8; //m/s^2 Acceleration of gravity
        const double INITIAL_VELOCITY = 0; //m/s Initial velocity of ball
        const double FLAG_HEIGHT = 0; //meters known height of flag object
        double distance= 0; //Variable for holding distance
        double height = 0; //Variable holding height to bottom of flag
        
    public:
        bool inRange = false;
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
            if (pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0)) < 0){
                inRange = false;
                return -10.0;
            }
            inRange = true;
            return (float)(atan((pow(v,2.0) - sqrt(pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0))))/(g*d) + offset));//calculates angle required, casting between numbers where needed
        }
        bool checkForHit(){
            return false;
        }
};
class BallLauncher : private Pid {
    private:
        
        int htzIndex = 0;//Number of flags in horizontal target zone
        const double pi = 3.141592;//Pi
        double toRad(double degrees){//Converts degrees to radians
            return degrees * (pi/180.0);
        }
        double toDeg(double radians){//converts radians to degrees
            return radians * (180.0/pi);
        }
        
        Flag htzFlags[9];
        //The following values are for the vision sensor camera
        const float FOV = 60;//field of view
        const float FOCAL_LENGTH = 320/tan(toRad(FOV/2));//Focal length of the camera based on field of view 
        const int htzMax = 205;//Upper limit of horizontal target zone
        const int htzMin = 195;//Lower limit of horizontal target zone
        float angleAtPoint(int y){//calculates the verticle angle to a specific point on the camera using the field of view and focal length
            float offset = 0;//Offset for systematic error
            int yP = -y + 320;//Makes center the origin
            return (float)(toDeg(atan((double)(yP/FOCAL_LENGTH)))) + offset;//Returns angle at point
        }
        
        void runAngleMotor(int angle){//Runs motor to set launcher to an angle
            /*kP = 2;
            kD = 0;
            kI = 0;
            setPoint = angle;
            float fix = pidCalc(gyroLauncherSet.value(gyroLauncher.value(vex::rotationUnits::deg)));
            if (fix > 100){
                return;
            }
            if (fix < -100){
                return;
            }*/
            robotMain.Screen.clearScreen();
            robotMain.Screen.setCursor(1,0);
            robotMain.Screen.print("Current Angle: %d", gyroLauncherSet.value(gyroLauncher.value(vex::analogUnits::range12bit)));
            robotMain.Screen.newLine();
            robotMain.Screen.print("Required Angle: %d", angle);
        }
    public:
        BallLauncher(){//Constructor, just sets gyro to 
            gyroLauncherSet.setValues(getAccelTiltAngle(), gyroLauncher.value(vex::analogUnits::range12bit), true);
            kP = 0.0;
            kI = 0.0;
            kD = 0.0;
        }
        int flagX[9];
        void scanForFlags(){//
            if (colorRed){
                visLauncher.takeSnapshot(SIG_FLAG_BLUE);
            } else {
                visLauncher.takeSnapshot(SIG_FLAG_RED);
            }
            htzIndex = 0;
            for (int i = 0; i < visLauncher.objectCount; i++){
                if (visLauncher.objects[i].centerX < htzMax && visLauncher.objects[i].centerX > htzMin){
                    float beta = angleAtPoint(visLauncher.objects[i].originX);
                    float alpha = angleAtPoint(visLauncher.objects[i].originX + visLauncher.objects[i].width);
                    htzFlags[htzIndex].calculateDistance(alpha, beta);
                    htzIndex ++;
                }
                flagX[i] = visLauncher.objects[i].centerX;
            }
        }
        void targetSpecificFlag(){
            float angles[htzIndex][2];
            for (int i = 0; i < htzIndex; i++){//Calculate the difference between the needed angle and gyro angle for all flags in the htz
                angles[i][0] = fabs(htzFlags[i].calculateRequiredAngle() - (float)gyroLauncherSet.value(gyroLauncher.value(vex::rotationUnits::deg)));
                if (htzFlags[i].inRange){
                    angles[i][1] = 1;
                } else {
                    angles[i][1] = 0;
                }
            }
            int shortestAngle = -1;//set the first angle to the shortest angle
            for (int i = 0; i < htzIndex; i++){
                if (shortestAngle == -1 && angles[i][1] == 1){
                    shortestAngle = i;
                    continue;
                }
                if (angles[i] < angles[shortestAngle] && angles[i][1] == 1){//If any other angle is closer to the gyro angle, set that to the closest angle
                    shortestAngle = i;
                }
            }
            if (shortestAngle != -1){
                runAngleMotor((int)htzFlags[shortestAngle].calculateRequiredAngle() - 44);//Run the motor to reach selected angle.
            } else {
                robotMain.Screen.clearScreen();
            }
        }
};
