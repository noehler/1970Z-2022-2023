#include "Autons/autonSetup.h"
#include "main.h"

autonTypes_t autonType = noAuton;
//0 is blue, 1 is red, 2 is driver skill
int color = true;

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
void shootdisks(void *mc, int overallStartTime, bool calibrapePos, int number){
    if (number == 4){
        number = sensing.robot.magFullness;
    }
    ((motorControl_t*) mc)->intakeRunning = 0;
    sensing.robot.turretLock = false;
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
    while(millis() - starttime <6000 && millis() - overallStartTime < 55000){
        //time out
        if (calibrapePos){
            gpsIntegX+=sensing.robot.GPSxpos;
            gpsIntegY+=sensing.robot.GPSxpos;
            loops++;
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
        sensing.robot.odoxpos = gpsIntegX/loops;
        sensing.robot.odoxpos = gpsIntegY/loops;
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

void intakeWaitForDiscs(void *mc, int maxTime, int overallStartTime, int goalAmt){
    int startTime = millis();
    while(distto(((motorControl_t*) mc)->move.moveToxpos, ((motorControl_t*) mc)->move.moveToypos) > ((motorControl_t*) mc)->move.tolerance && sensing.robot.magFullness < goalAmt && millis()-startTime < maxTime && millis() - overallStartTime < 55000){
        delay(10);
    }
}

void waitRotate(void *mc, int maxTime, int overallStartTime){
    int startTime = millis();
    while(fabs(sensing.robot.angle-((motorControl_t*) mc)->HeadingTarget)+fabs(sensing.robot.velW)>= ((motorControl_t*) mc)->move.errtheta && millis() - startTime <maxTime && millis() - overallStartTime < 55000){
        delay(10);
    }
}
