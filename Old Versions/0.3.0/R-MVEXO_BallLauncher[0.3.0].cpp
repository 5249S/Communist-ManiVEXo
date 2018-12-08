/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Ball Launcher               */
/*                Version 0.3.0               */
/*--------------------------------------------*/
class Flag {
    private:
        const double GRAVITY = -9.8; 
        const double INITIAL_VELOCITY = 0;
        const double FLAG_HEIGHT = 0.14;
        double distance= 0; 
        double height = 0;
        
    public:
        bool inRange = false;
        
        void calculateDistance(double alpha, double beta){
            int offset = 0; //
            distance = (FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (tan(alpha));
        }
        double calculateRequiredAngle(){
            double v = INITIAL_VELOCITY;
            double g = GRAVITY;
            double h = height + FLAG_HEIGHT/2;
            double d = distance;
            double offset = -11;
            if (pow(v,4.0)-g(g*pow(d,2.0)-2*h*pow(v, 2.0)) < 0){
                inRange = false;
                return -10.0;
            }
            inRange = true;
            return (atan((pow(v,2.0) - sqrt(pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0))))/(g*d)) + offset);
        }
        bool checkForHit(){
            return false;
        }
};
class BallLauncher {
    private:
        
        int htzIndex = 0;
        const double pi = 3.141592;
        double toRad(double degrees){
            return degrees * (pi/180.0);
        }
        double toDeg(double radians){
            return radians * (180.0/pi);
        }
        
        Flag htzFlags[9];
        const double FOV = 60;
        const double FOCAL_LENGTH = 320/tan(toRad(FOV/2));
        const int htzMax = 205;
        const int htzMin = 195;
        double angleAtPoint(int y){
            double offset = 0;
            int yP = -y + 320;
            return (atan((yP/FOCAL_LENGTH))) + offset;
        }
    public:
        
        int flagX[9];
        int scanForFlags(){//
            if (colorRed){
                visLauncher.takeSnapshot(SIG_FLAG_BLUE);
            } else {
                visLauncher.takeSnapshot(SIG_FLAG_RED);
            }
            htzIndex = 0;
            for (int i = 0; i < visLauncher.objectCount; i++){
                if (visLauncher.objects[i].centerY < htzMax && visLauncher.objects[i].centerY > htzMin){
                    double beta = angleAtPoint(visLauncher.objects[i].originX);
                    double alpha = angleAtPoint(visLauncher.objects[i].originX + visLauncher.objects[i].width);
                    htzFlags[htzIndex].calculateDistance(alpha, beta);
                    htzIndex ++;
                }
                flagX[i] = visLauncher.objects[i].centerX;
            }
            return htzIndex;
        }
        double targetSpecificFlag(){
            double angles[htzIndex][2];
            for (int i = 0; i < htzIndex; i++){
                angles[i][0] = fabs(toDeg(htzFlags[i].calculateRequiredAngle()) - getAccelTiltAngle());
                if (htzFlags[i].inRange){
                    angles[i][1] = 1;
                } else {
                    angles[i][1] = 0;
                }
            }
            int shortestAngle = -1;
            for (int i = 0; i < htzIndex; i++){
                if (shortestAngle == -1 && angles[i][1] == 1){
                    continue;
                }
                if (angles[i] < angles[shortestAngle] && angles[i][1] == 1){
                    shortestAngle = i;
                }
            }
            if (shortestAngle != -1){
                return toDeg(htzFlags[shortestAngle].calculateRequiredAngle());
            } else {
                return -1;
            }
        }
};
