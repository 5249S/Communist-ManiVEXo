class Flag {
    private:
        const float GRAVITY = 9.8 //m/s^2
        const float INITIAL_VELOCITY = 0 //m/s
        const float FLAG_HEIGHT = 0 //meters
        float distance = 0;
        float height = 0;
        void calculateDistance(double alpha, double beta){
            int offset = 0;
            distance = (float)(FLAG_HEIGHT/(tan(beta)-tan(alpha)) + offset);
            height = distance * (float)(tan(alpha));
        }
    public:
        float calculateRequiredAngle(float alpha, float beta){
            float v = INITIAL_VELOCITY;
            float g = GRAVITY;
            float h = height + FLAG_HEIGHT/2;
            return (float)(atan((v^2 - sqrt(v^4-g(g*distance^2-2*h*v^2)))/g*distance));
        }
        bool checkForHit(float alpha, float beta){
            
        }
};
class Launcher {
    private:
        const double pi = 3.141592;
        double toRad(double degrees){
            return degrees * (pi/180.0);
        }
        double toDeg(double radians){
            return radians * (180.0/pi);
        }
        Flag htzFlags[9];
        const float FOV = 47.0;//field of view
        const float FOCAL_LENGTH = 200/tan(toRad(23.5));
        float angleAtPoint(int y){
            float offset = 0;
            int yP = y - 200;
            return (float)(atan((double)(yP/FOCAL_LENGTH)) + offset;
        }
    public:
      void scanForFlags(){
          
      }
      void targetSpecificFlag(){

      }
};
