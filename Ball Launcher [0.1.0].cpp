class Flag {
    private:
        const float GRAVITY = -9.8 //m/s^2 Acceleration of gravity
        const float INITIAL_VELOCITY = 0 //m/s Initial velocity of ball
        const float FLAG_HEIGHT = 0 //meters known height of flag object
        float distance = 0; //Variable for holding distance
        float height = 0; //Variable holding height to bottom of flag
        
    public:
        //Equations are based on inmatic formulas
        void calculateDistance(double alpha, double beta){//Takes angle values and sets instance varibles to distance an height
            int offset = 0; //
            distance = (float)(FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (float)(tan(alpha));
        }
        float calculateRequiredAngle(){//Returns angle required to hit flag
            float v = INITIAL_VELOCITY;//variables to hold values, for simplicity
            float g = GRAVITY;
            float h = height + FLAG_HEIGHT/2;//height of middle of flag
            int offset = 0;//Offset varible to adjust for systematic error
            return (float)(atan((v^2 - sqrt(v^4-g(g*distance^2-2*h*v^2)))/g*distance) + offset);//calculates angle required, casting between numbers where needed
        }
        bool checkForHit(){
            
        }
};
class Launcher {
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
                return (int)(toDeg(-asin((double)Z/abs(Z)))) + offset);//gives back either 0 or 90 degrees
            }
            return (int)(toDeg(-atan((double)(Z/Y))) + offset);//Calculates angle
        }
        Flag htzFlags[9];
        //The following values are for the vision sensor camera
        const float FOV = 47.0;//field of view
        const float FOCAL_LENGTH = 200/tan(toRad(23.5));//Focal length of the camera based on field of view 
        const int htzMax = 340;//Upper limit of horizontal target zone
        const int htzMin = 300;//Lower limit of horizontal target zone
        float angleAtPoint(int y){//calculates the verticle angle to a specific point on the camera using the field of view and focal length
            float offset = 0;//Offset for systematic error
            int yP = y - 200;//Makes center the origin
            return (float)(toDeg(atan((double)(yP/FOCAL_LENGTH))) + offset;//Returns angle at point
        }
        Pid pidLauncher;
        void runPid(float angle){//Runs motor to set launcher to an angle
            
        }
    public:
        Launcher(){//Constructor, just sets gyro to 
            gyroLauncherSet.setValues(accelToGyro(), gyroLauncher.value(vex::rotationUnits::deg), false);
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
            smallestAngleDifference = 100;
            current 
            for (int i = 0; i < htzIndex; i++){
                
            }
        }
};
