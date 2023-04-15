#include "Autons/autonSetup.h"
#include "GUI.h"
#include "main.h"
#include "output.h"
#include "pros/misc.hpp"
#include "robotConfig.h"

autonTypes_t autonType = noAuton;

int color = true;//0 is blue, 1 is red, 2 is driver skill

bool updateScreen[2] = {1,1};

//macro for pythagorean theorum because used so much in macros
double distto (double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
}

//sets all variables for correct use of MoveTo function
void moveto(void *mc, double xTo, double yTo, double speedLimit, double tolerance, double errtheta, int forward){
    char temp[50];
    sprintf(temp,"move To (%f, %f)", xTo,yTo);
    logMessage(temp);
    //moveto frame parameters
    ((motorControl_t*) mc)->driveType = 0;
    ((motorControl_t*) mc)->move.moveToxpos = xTo;
    ((motorControl_t*) mc)->move.moveToypos = yTo;
    ((motorControl_t*) mc)->move.speed_limit = speedLimit;
    ((motorControl_t*) mc)->move.tolerance = tolerance;
    ((motorControl_t*) mc)->move.errtheta = errtheta;
    ((motorControl_t*) mc)->move.moveToforwardToggle = forward;
}

//wait until shooter is lined up, robot is still, speed is right, or timout to shoot
void shootdisks(void *mc, int overallStartTime, bool calibrapePos, int number, int overallMaxTime, int maxTime){
    ((motorControl_t*) mc)->driveType = 1;
    ((motorControl_t*) mc)->HeadingTarget = goalAngle;
    if (number == 4){
        number = sensing.robot.magFullness;
    }
    ((motorControl_t*) mc)->autoAim = true;
    delay(20);
    int starttime = millis();
    double gpsIntegX = 0;
    double gpsIntegY = 0;
    double loops = 0;
    ((motorControl_t*) mc)->updatedAD = false;
    while(millis() - starttime < maxTime/* && millis() - overallStartTime < overallMaxTime*/){
        ((motorControl_t*) mc)->HeadingTarget = goalAngle;
        //time out
        if (calibrapePos){
            if (sensing.GPS_sensor.get_error() < 0.012){
                gpsIntegX+=sensing.robot.GPSxpos;
                gpsIntegY+=sensing.robot.GPSxpos;
                loops++;
            }
        }
        if (((motorControl_t*) mc)->updatedAD && fabs(((motorControl_t*) mc)->angdiff)<3 && (fabs(((motorControl_t*) mc)->diffFlyWheelW)) < 5 && fabs(sensing.robot.velX)+fabs(sensing.robot.velY) + fabs(sensing.robot.turvelw)*2 + fabs(sensing.robot.angAccel)*2 < 30){
            logMessage("turret good exit");
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (calibrapePos){
        sensing.robot.xpos = gpsIntegX/loops;
        sensing.robot.xpos = gpsIntegY/loops;
    }
    if (number!=1){//second batch
    ((motorControl_t*) mc)->raiseAScore(3);
        logMessage("shoot 3");
    } else {
    ((motorControl_t*) mc)->raiseAScore(1);
        logMessage("shoot 1");
    }
    delay(300);
    ((motorControl_t*) mc)->autoAim = false;
}

//start intake, lock turret
void intake(void *mc){
    ((motorControl_t*) mc)->intakeRunning = 1;
    ((motorControl_t*) mc)->autoAim = false;
}

//run drive at set voltage
void movevoltage(void *mc, double L, double R){
    
    ((motorControl_t*) mc)->driveType = 2;
    ((motorControl_t*) mc)->rightSpd = R;
    ((motorControl_t*) mc)->leftSpd = L;
}

//set parameters to rotate to set angle
void rotateto(void *mc,double ang, double range){
    logMessage("rotate");
    ((motorControl_t*) mc)->driveType = 1;
    ((motorControl_t*) mc)->HeadingTarget = ang;
    ((motorControl_t*) mc)->move.errtheta = range;
}

//macro to wait for correct position or time
void waitPosTime(void *mc, int maxTime, int overallStartTime,
                int overallMaxTime) {
    int startTime = millis();
    ((motorControl_t*) mc)->move.resetMoveTo = false;
    while (millis() - startTime < maxTime && ((motorControl_t*) mc)->move.resetMoveTo == false &&
            millis() - overallStartTime < overallMaxTime) {
        delay(20);
    }
}

//wait until magazine is filled or time is reached
void intakeWaitForDiscs(void *mc, int maxTime, int overallStartTime, int goalAmt, int overallMaxTime){
    logMessage("collecting discs");
    int startTime = millis();
    while(distto(((motorControl_t*) mc)->move.moveToxpos, ((motorControl_t*) mc)->move.moveToypos) > ((motorControl_t*) mc)->move.tolerance && sensing.robot.magFullness < goalAmt && millis()-startTime < maxTime && millis() - overallStartTime < overallMaxTime){
        delay(10);
    }
}

//wait until specific angle is reached or time is reached
void waitRotate(void *mc, int maxTime, int overallStartTime, int overallMaxTime){
    int startTime = millis();
    while(fabs(sensing.robot.angle-((motorControl_t*) mc)->HeadingTarget)+fabs(sensing.robot.velW)>= ((motorControl_t*) mc)->move.errtheta && millis() - startTime <maxTime && millis() - overallStartTime < overallMaxTime){
        delay(10);
    }
}

//updates screen to match current selection of auton
void AutonSelector(void){    
    while(competition::is_disabled()){
        
        if (updateScreen[0] == true){
            delay(50);
            master.clear();
            delay(50);
            if (color == 0){
                master.print(0, 0, "blue");
            }else if (color == 1){
                master.print(0, 0, "red");
            }else{
                master.print(0, 0, "skills");
            }
            delay(50);
            if (autonType == winPointClose){
                master.print(1, 0, "close wp");
            }else if (autonType == winPointFar){
                master.print(1, 0, "far wp");
            }else if (autonType == winPointBoth){
                master.print(1, 0, "full wp");
            }else if (autonType == noAuton){
                master.print(1, 0, "no Auton");
            }else{
                master.print(1, 0, "skills");
            }

            delay(50);
            master.print(2, 0, "Field: %d", current_field);

            updateScreen[0] = false;
        }
        delay(30);
    }
}