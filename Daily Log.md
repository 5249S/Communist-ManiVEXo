# Daily Log

All significant changes made to the software for 5249S will be documented here

## Initial Setup
### April 17, 2018
- Created repository for storing the code 
- Added reference files from last year's competition for driving control
- Made a c++ document with driving code from last year
## C-MVEXO [0.0.0]
### April 30, 2018
- Created the main file for the robot
- Put in a general PID calculator
### May 1, 2018
- Added last year's driving system
- Added driving for an H-drive chassis
### May 5, 2018
- Added driving for an X-drive chassis
- X-Drive Probably will not be used as only 8 V5 motors are allowed
### May 7, 2018
- Researched the new V5 Vision Sensor and researched ways to locate target using sensor for ball launcher
### May 9, 2018
- Finished initial ball launcher calculations
### May 20, 2018
- Began research on drag and looking to calculate projectile path incorperating drag
- Drag will probably not be used as it will most likely prove to be too precise
### May 30, 2018
- Finalized calculations for ball launcher including vision sensor distance calculations and projectile path with and without drag
### May 31, 2018
- Added code for a ball launcher that calculates if it will hit a target
- Commented different parts of the code
- Put ball launcher code in a seperate file, it will be added in future versions
### June 3, 2018
- Found calculations for auto angle adjustment
### June 6, 2018
- Changed the PID calculator format so it can be easier used with the object oriented part of c++
### August 14, 2018
- Overlooked newly released API for Vex Coding Studio
### August 18, 2018
- Integrated VEX OS into the main program
- Version 0.0.0 is ready to be tested
### August 21, 2018
- Successfully compiled version 0.0.0, ready for testing
## C-MVEXO [0.1.0]
### August 19, 2018
- Began working on versions 0.1.0
### September 14, 2018
- Researched V5 Vision Sensor and determined method for locating flags
### September 16, 2018
- Finished first draft of 0.1.0
### September 18, 2018
- Revised 0.1.0 to reduce lines of code
### September 24, 2018
- Uploaded first draft of 0.1.0
- Updated changelog for 0.1.0
### September 25, 2018
- Started new version of ball launcher code
### September 28, 2018
- Configured gyro and accelerometer configurations
### September 30, 2018
- Finished and fixed error for ball launcher code
- Reorganized code
### October 10, 2018
- Added lift control
- Fixed syntax mistakes
### October 11, 2018
- Began 0.1.1, fixing update problems
### October 16, 2018
- Finished 0.1.1, fixed problems
## MVEXO [0.2.0]
### October 18, 2018
- Began work on live diagnostics system
### October 24, 2018
- Finished live diagnostics system
- Fixed field control problems
## R-MVEXO [0.3.0]
### November 8, 2018
- Added driver control for ball launcher and claw
### November 13, 2018
- Re-wired robot
- Updated code for targeting system 
- Began writing PID methods for auton driving
### November 14, 2018
- Calibrated accelerometer
### November 15, 2018
- Fixed tilt angle problem
- Finished code for first test of targetting system
### November 21, 2018
- Began testing of Ball Launcher program
### November 23, 2018
- Successfully aimed ball launcher using targetting system
- Began tuning PIDs for driving in auton
### November 24, 2018
- Created a PID for auto-aiming the ball launcher at flags
- Wrote a basic auton for testing 
- Wrote code for an indicator light targeting guide
### November 26, 2018
- Created a horizontal alignment program for the targeting system
### November 28, 2018
- Fixed horizontal alignment program
- Finalized 0.3.0
### December 1, 2018
- First Competition
- Wrote first auton
#### Successes
- Target system was successful
- Won Innovate award for the ball launcher autotarget system
#### Failures
- Ball launcher wasn't able to work well with the ball intake
- No auton besides just using the preload to hit flags
## R-MVEXO [1.0.0]
### December 8, 2018
- Began work on 1.0.0
- Began work on a navigation system for autons
### December 9, 2018
- Began coding an acceleration tracking system for navigation
### December 12, 2018
- Wrote methods for using the tracking system in autons
### December 20, 2018
- Began debugging the tracking system code
### December 22, 2018
- Found our gyroscope was shorting out and overheating
- Abandoned tracking system because the accelerometer sensor was too noisy to integrate its values accurately
- Started working with motor encoder count commands
- Began first auton with encoder counts
### December 23, 2018
- Coded a lift positioning system to help the driver position the lift to place caps on poles
- Coded a toggle for flipping the claw
### December 25, 2018
- Continued coding a skills auton 
### December 27, 2018
- Coded a skills auton which places a flag on a pole and fires a ball at a flag
- Tested auton successfully
### December 28, 2018
- Extended the auton for skills to 12 points, not yet tested
- Coded a game auton to ideally score 11 points, not tested
- Coded an alternative auton that scores 4 points, not tested
### January 7, 2019
- Happy New Year
- Tested each auton once
- Fixed mistakes in initial tests
