#include "Autons/autonSetup.h"
#include "main.h"
#include "pros/misc.hpp"

autonTypes_t autonType = noAuton;
//0 is blue, 1 is red, 2 is driver skill
int color = true;
bool updateScreen = true;

void autonNotSetup(void){
    
}

double distto (double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
 }


void moveto(void *mc, double xTo, double yTo, double speedLimit, double tolerance, double errtheta, int forward){
    //moveto frame parameters
    ((motorControl_t*) mc)->driveType = 0;
    ((motorControl_t*) mc)->move.moveToxpos = xTo;
    ((motorControl_t*) mc)->move.moveToypos = yTo;
    ((motorControl_t*) mc)->move.speed_limit = speedLimit;
    ((motorControl_t*) mc)->move.tolerance = tolerance;
    ((motorControl_t*) mc)->move.errtheta = errtheta;
    ((motorControl_t*) mc)->move.moveToforwardToggle = forward;
}

double pickPos(double posInput, int run){
    if (run == 0){
        return posInput;
    }
    else{
        return 144 - posInput;
    }
}
void shootdisks(void *mc, int overallStartTime, bool calibrapePos, int number, int overallMaxTime, int maxTime){
    if (number == 4){
        number = sensing.robot.magFullness;
    }
    ((motorControl_t*) mc)->intakeRunning = 2;
    sensing.robot.turretLock = false;
    delay(200);
    ((motorControl_t*) mc)->intakeRunning = 0;
    delay(20);
    if (number!=1){
    ((motorControl_t*) mc)->discCountChoice = 2;
    } else {
    ((motorControl_t*) mc)->discCountChoice = 1;
    }
    int starttime = millis();
    double gpsIntegX = 0;
    double gpsIntegY = 0;
    double loops = 0;
    ((motorControl_t*) mc)->updatedAD = false;
    while(millis() - starttime <maxTime && millis() - overallStartTime < overallMaxTime){
        //time out
        if (calibrapePos){
            if (sensing.GPS_sensor.get_error() < 0.012){
                gpsIntegX+=sensing.robot.GPSxpos;
                gpsIntegY+=sensing.robot.GPSxpos;
                loops++;
            }
        }
        if (((motorControl_t*) mc)->updatedAD && fabs(((motorControl_t*) mc)->angdiff)<3 && (fabs(((motorControl_t*) mc)->diffFlyWheelW) + fabs(((motorControl_t*) mc)->diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY) + fabs(sensing.robot.turvelw)*2 +  + fabs(sensing.robot.angAccel)*2< 30){
            std::cout<<"good turret"<<"\n";
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
    } else {
    ((motorControl_t*) mc)->raiseAScore(1);
    }
    delay(300);
    sensing.robot.turretLock = true;
}
void intake(void *mc){
    ((motorControl_t*) mc)->intakeRunning = 1;
    sensing.robot.turretLock = true;
}
void movevoltage(void *mc, double L, double R){
    
    ((motorControl_t*) mc)->driveType = 3;
    ((motorControl_t*) mc)->rightSpd = R;
    ((motorControl_t*) mc)->leftSpd = L;
}
void rotateto(void *mc,double ang, double range){
    
    ((motorControl_t*) mc)->driveType = 1;
    ((motorControl_t*) mc)->HeadingTarget = ang;
    ((motorControl_t*) mc)->move.errtheta = range;
}

void intakeWaitForDiscs(void *mc, int maxTime, int overallStartTime, int goalAmt, int overallMaxTime){
    int startTime = millis();
    while(distto(((motorControl_t*) mc)->move.moveToxpos, ((motorControl_t*) mc)->move.moveToypos) > ((motorControl_t*) mc)->move.tolerance && sensing.robot.magFullness < goalAmt && millis()-startTime < maxTime && millis() - overallStartTime < overallMaxTime){
        delay(10);
    }
}

void waitRotate(void *mc, int maxTime, int overallStartTime, int overallMaxTime){
    int startTime = millis();
    while(fabs(sensing.robot.angle-((motorControl_t*) mc)->HeadingTarget)+fabs(sensing.robot.velW)>= ((motorControl_t*) mc)->move.errtheta && millis() - startTime <maxTime && millis() - overallStartTime < overallMaxTime){
        delay(10);
    }
}

void AutonSelector(void){    
    while(competition::is_disabled()){
        /*if (master.get_digital(DIGITAL_A)){
            autonType = winPointBoth;
            updateScreen = true;
        }

        if (master.get_digital(DIGITAL_B)){
            autonType = winPointClose;
            updateScreen = true;
        }

        if (master.get_digital(DIGITAL_X)){
            autonType = winPointClose;
            updateScreen = true;
        }

        if (master.get_digital(DIGITAL_Y)){
            autonType = noAuton;
            updateScreen = true;
        }

        if (master.get_digital(DIGITAL_DOWN)){
            autonType = skillsAuton;
            updateScreen = true;
        }

        if (master.get_digital(DIGITAL_UP)){
            color+=1;
            if (color == 3){
                color = 0;
            }
            updateScreen = true;
        }*/
        
        if (updateScreen == true){
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

            updateScreen = false;
        }
        delay(30);
    }
}