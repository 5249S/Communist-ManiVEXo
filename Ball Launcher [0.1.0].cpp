class Flag {
    private:
        const float GRAVITY = 9.8 //m/s^2
        const float INITIAL_VELOCITY = 0 //m/s
        const float FLAG_HEIGHT = 0 //meters
        float distance = 0;
        float height = 0;
        
    public:
        void calculateDistance(double alpha, double beta){
            int offset = 0;
            distance = (float)(FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (float)(tan(alpha));
        }
        float calculateRequiredAngle(){
            float v = INITIAL_VELOCITY;
            float g = GRAVITY;
            float h = height + FLAG_HEIGHT/2;
            int offset = 0;
            return (float)(atan((v^2 - sqrt(v^4-g(g*distance^2-2*h*v^2)))/g*distance) + offset);
        }
        bool checkForHit(){
            
        }
};
class Launcher {
    private:        
        int htzIndex = 0;
        const double pi = 3.141592;
        double toRad(double degrees){
            return degrees * (pi/180.0);
        }
        double toDeg(double radians){
            return radians * (180.0/pi);
        }
        int accelToGyro(){
            int Z = accelLauncherZ.value(vex::analogUnits::range12bit);
            int Y = accelLauncherY.value(vex::analogUnits::range12bit);
            int offset = 0;
            if (Y == 0){
                return (int)(toDeg(-asin((double)Z))) + offset);
            }
            return (int)(toDeg(-atan((double)(Z/Y))) + offset);
        }
        Flag htzFlags[9];
        const float FOV = 47.0;//field of view
        const float FOCAL_LENGTH = 200/tan(toRad(23.5));
        const int htzMax = 340;
        const int htzMin = 300;
        float angleAtPoint(int y){
            float offset = 0;
            int yP = y - 200;
            return (float)(atan((double)(yP/FOCAL_LENGTH)) + offset;
        }
        Pid pidLauncher;
        void runPid(float angle){
            
        }
    public:
        Launcher(){
            gyroLauncherSet.setValues(accelToGyro(), gyroLauncher.value(vex::rotationUnits::deg), false);
        }
        int flagX[9];
        void scanForFlags(){
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
