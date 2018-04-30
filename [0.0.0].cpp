class driveMethods {}
class pid: public driveMethods {
    public:
        struct pidCalcReturn {
            float adjust;
            float lastError;
            float integral;
        }
        pidCalcReturn pidCalc(float processVar, float setPoint = 0, float prevError = 0, float pidIntegral = 0, float kP = 0.0, float kI = 0.0, float kD = 0.0){
            
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
            if (abs(pidIntegral) < 50) {
                    pidIntegral = pidError - (pidError-prevError)/2 + pidIntegral;
            } else {
                    pidIntegral = 0;
            }

            //use pid structure to return values
            pidCalcReturn returnValues;
            returnValues.adjust = pidDerivative + pidProportional + kI * pidIntegral;
            returnValues.lastError = pidError;
            returnValues.integral = pidIntegral;

            //return adjustment, new error, and pidIntegral
            return returnValues;
        }
}
}
class roboMethods: public pid {

}
