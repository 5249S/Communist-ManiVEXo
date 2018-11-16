/*--------------------------------------------*/
/*                    5249S                   */
/*              Robotic ManiVEXo              */
/*                Ball Launcher               */
/*                Version 0.3.0               */
/*--------------------------------------------*/
class Flag {
    private:
        const double GRAVITY = -9.8; //m/s^2 Acceleration of gravity
        const double INITIAL_VELOCITY = 0; //m/s Initial velocity of ball
        const double FLAG_HEIGHT = 0.14; //meters known height of flag object
        double distance= 0; //Variable for holding distance
        double height = 0; //Variable holding height to bottom of flag
        
    public:
        bool inRange = false;
        //Equations are based on inmatic formulas
        void calculateDistance(double alpha, double beta){//Takes angle values and sets instance varibles to distance an height
            int offset = 0; //
            distance = (FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (tan(alpha));
        }
        double calculateRequiredAngle(){//Returns angle required to hit flag in radians
            double v = INITIAL_VELOCITY;//variables to hold values, for simplicity
            double g = GRAVITY;
            double h = height + FLAG_HEIGHT/2;//height of middle of flag
            double d = distance;
            double offset = -11;//Offset varible to adjust for systematic error
            if (pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0)) < 0){
                inRange = false;
                return -10.0;
            }
            inRange = true;
            return (atan((pow(v,2.0) - sqrt(pow(v,4.0)-g*(g*pow(d,2.0)-2*h*pow(v, 2.0))))/(g*d)) + offset);//calculates angle required, casting between numbers where needed
        }
        bool checkForHit(){
            return false;
        }
};
class BallLauncher {
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
        const double FOV = 60;//field of view
        const double FOCAL_LENGTH = 320/tan(toRad(FOV/2));//Focal length of the camera based on field of view 
        const int htzMax = 205;//Upper limit of horizontal target zone
        const int htzMin = 195;//Lower limit of horizontal target zone
        double angleAtPoint(int y){//calculates the verticle angle to a specific point on the camera using the field of view and focal length
            double offset = 0;//Offset for systematic error
            int yP = -y + 320;//Makes center the origin
            return (atan((yP/FOCAL_LENGTH))) + offset;//Returns angle at point
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
            for (int i = 0; i < htzIndex; i++){//Calculate the difference between the needed angle and gyro angle for all flags in the htz
                angles[i][0] = fabs(toDeg(htzFlags[i].calculateRequiredAngle()) - getAccelTiltAngle());
                if (htzFlags[i].inRange){
                    angles[i][1] = 1;
                } else {
                    angles[i][1] = 0;
                }
            }
            int shortestAngle = -1;//set the first angle to the shortest angle
            for (int i = 0; i < htzIndex; i++){
                if (shortestAngle == -1 && angles[i][1] == 1){
                    continue;
                }
                if (angles[i] < angles[shortestAngle] && angles[i][1] == 1){//If any other angle is closer to the gyro angle, set that to the closest angle
                    shortestAngle = i;
                }
            }
            if (shortestAngle != -1){
                return toDeg(htzFlags[shortestAngle].calculateRequiredAngle());//Run the motor to reach selected angle.
            } else {
                return -1;
            }
        }
};
