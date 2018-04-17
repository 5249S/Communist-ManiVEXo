/*------------------------------------------------------*/
/*driving code                                          */
/*Author: Cameron Ekeman                                */
/*Created: 2017-2-15                                    */
/*A driving algorithm for an H or U drive robot that    */
/*takes horizontal and vertical values from driver      */
/*joysticks and returns a power value to the motors     */
/*------------------------------------------------------*/

int powerLeft = 0;
int powerRight = 0;
y = vexRT[Ch3];
x = vexRT[Ch1];
//Deadzone Filter to prevent motor whining or drifting
if (y < 10 && y > -10) {
    y = 0;
}
if (x < 10 && x > -10) {
    x = 0;
}

//Left Motor Calculator
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

//Right Motor Calculator
if (y >= 0) {
    if (x <= 0) {
        if (y >= -x) {
            powerRight = y;
        } else {
            powerRight = -x;
        }
    } else {
        powerRight = y - x;
    }
} else {
    if (x >= 0) {
        if (y <= -x) {
            powerRight = y;
        } else {
            powerRight = -x;
        }
    } else {
        powerRight = y - x;
    }
}

//Use power variables to control the motors
motor[leftFront] = powerLeft;
motor[rightBack] = powerRight;
motor[leftBack] = powerLeft;
motor[rightFront] = powerRight;
