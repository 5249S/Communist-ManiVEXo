//Code for ball launcher, will not be used in version 0.0.0 but will be added to future versions when ball launcher is constructed
class Launcher { //Class for measuring and calculating whether the robot's ball launcher in it's current state will hit a flag
    private: 
        //All measurements use SI units
        const float GRAVITY = 9.8; //meters per second^2
        const float INITIAL_VELOCITY = 0; //To be calculated
        const float FLAG_HEIGHT = 0; //Centimeters
        const int BALL_MASS = 55; //mass in grams
        const float k = 0; //Constant of drag
        float distance = 0;
        float height = 0;
        float pi = 3.1415;
        float e = 2.7182;
        senseVision(){//Uses vision sensor to calculate horizontal distance away from the flag and height from 'horizon' to bottom of flag
            float alpha;
            float beta;
            
            distance = FLAG_HEIGHT/(tan(beta) - tan(alpha);//Uses trig to determine values an changes instance variables to them
            height = distance * tan(alpha);
        }
        
    public:
        ballToFlagCheck(float x = distance){//Using kinematic formulas, use the launcher angle to determine if the ball will hit the flag
            float a = INITIAL_VELOCITY * cos(gyroLauncher.angle() * pi / 180);//Calculate horizontal velocity component
            float b = INITIAL_VELOCITY * sin(gyroLauncher.angle() * pi / 180);//Calculate verticle velocity component
            
            float y = -(GRAVITY * x^2)/(2 * a^2) + b * x/a; //Calculate what the height will be at the given distance
            if(y >= height + 0.5 && y >= height + FLAG_HEIGHT - 0.5){//Determine if that height is at the flag's height
                return true;
            }
            return false;
        }
        ballToFlagCheckDrag(float x = distance){//Using kinematic formulas and using an equation for drag, F= -k*v, use the launcher angle to determine if the ball will hit the flag
            float a = INITIAL_VELOCITY * cos(gyroLauncher.angle() * pi / 180);//Calculate horizontal velocity component
            float b = INITIAL_VELOCITY * sin(gyroLauncher.angle() * pi / 180);//Calculate verticle velocity component
            if(k == 0){//If constant of drag is 0, return regular kinematic formula answer to prevent math error
                return ballToFlagCheck();
            }
            float mass = BALL_MASS / 1000;//convert mass to kilograms
            float t = (BALL_MASS/k) * log(1 - x/((BALL_MASS/k)*a)); //Calculate time until distance reached
            float y = -(GRAVITY * mass / k)*t + (mass/ k)*(b + (GRAVITY * mass/k))*(1 - e^(-k*t/ mass));//Calculate height at that time
            if(y >= height + 0.5 && y >= height + FLAG_HEIGHT - 0.5){//Determine if that height is at the flag's height
                return true;
            }
            return false;
        }
        updateRemoteControlInterface(){}
    
}
