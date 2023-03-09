#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"
 double distto (double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
 }


void moveto(void *mc, double xTo, double yTo, double speedLimit = 100, double tolerance = 5, double errtheta = 5, int forward = 1){
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
void shootdisks(void *mc, bool calibrapePos = false, int number = 4){
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
    while(millis() - starttime <6000){
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
void rotateto(void *mc,double ang, double range = 3){
    
    ((motorControl_t*) mc)->driveType = 1;
    ((motorControl_t*) mc)->HeadingTarget = ang;
    ((motorControl_t*) mc)->move.errtheta = range;
}

void intakeWaitForDiscs(void *mc, int maxTime, int goalAmt = 3){
    int startTime = millis();
    while(distto(((motorControl_t*) mc)->move.moveToxpos, ((motorControl_t*) mc)->move.moveToypos) > ((motorControl_t*) mc)->move.tolerance && sensing.robot.magFullness < goalAmt && millis()-startTime < maxTime){
        delay(10);
    }
}
    //sensing.set_status(totalX/loops,totalY/loops,270,100, 0);
    //standard motion frame
    /*
    //turret control parameters
    sensing.robot.turretLock = true; //true -> lock, false -> unlock
    mc.raiseAScore(3 or 1 disks shoots);
    mc.discCountChoice = 1 for single, 2 for automatic detection;

    //intake control parameter
    mc.intakeRunning = 1;

    //rotateto frame prarmeters
    mc.driveType = 1;
    mc.HeadingTarget = 270; 

    */
    //starting controller threads

void waitRotate(void *mc, int maxTime){
    int startTime = millis();
    while(fabs(sensing.robot.angle-((motorControl_t*) mc)->HeadingTarget)+fabs(sensing.robot.velW)>= ((motorControl_t*) mc)->move.errtheta && millis() - startTime <maxTime){
        delay(10);
    }
}

void skillsAutonomous(void){
    
    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
    

    double startTime = millis();
    sensing.goalSpeed = 180;
    sensing.set_status(37,18,270,100, 0);
    sensing.goal.xpos = 124;
    sensing.goal.ypos = 20;
    sensing.robot.turretLock = true;
    movevoltage(&mc, 0, 0);
    delay(50);
    //move to roller
    mc.driveToRoller(2500);
    
    //move out of roller
    moveto(&mc, 37, 16,100,0,20,-1);                               //John: tolerance is bigger than exit condition potentially a stop point
    while(sensing.robot.ypos<16){
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 48, 23,100,5,10,-1);
    mc.waitPosTime(2500);
    //line up with disk and roller might need a hold on drive motor
    rotateto(&mc, 180);
    startTime = millis();
    waitRotate(&mc, 1000);

    //pick up disk
    moveto(&mc, 0, 23,35,10,3,1);                                  //John: potentially too big of error heading, but at the same time, the pid spin is wack so idk.
    intake(&mc);
    startTime = millis();
    while(sensing.robot.xpos>24 &&millis()-startTime<5000){
        delay(10);
    }

    //shooting batch 1
    movevoltage(&mc, 0,0);
    mc.intakeRunning = 0;                                           //John: between each shoots, time can be lowered to 200ms or lower 
    delay(200);
    shootdisks(&mc,0,1);
    delay(200);
    shootdisks(&mc,0,1);
    delay(200);
    shootdisks(&mc,0,1);
    
    //drive to roller
    moveto(&mc, 0, 23,100,10,5,1); 
    startTime = millis();
    while((sensing.robot.xpos>pickPos(18, 0) || sensing.robot.xpos<pickPos(18, 1))&&millis()-startTime<1000){//disguesting picpos
        delay(10);
    }
    //get the roller
    mc.intakeRunning = 0;
    mc.driveToRoller(  2500);

    //out of roller
    moveto(&mc, 20,23,50,5,10,-1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(18, 0) || sensing.robot.xpos>pickPos(18, 1))&&millis()-startTime<1000){
        //time out
        //robot pos check
        delay(10);
    }

    //first triple stack
    intake(&mc);
    moveto(&mc, 46,35,30,3, 5,1);                                        //John: maybe errtheta too big, and maybe too far of distance to travel, I would like to test some manuver to knock over stacks
    
    intakeWaitForDiscs(&mc,3000);
    if(sensing.robot.magFullness != 3){
        moveto(&mc, 40,33,100,5,10,-1);
        mc.waitPosTime(3000);

        moveto(&mc, 48,40,30,3);
        intakeWaitForDiscs(&mc,3000);
    }
    delay(600);

    //shoot first triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc);
    sensing.robot.turretLock = true;
    delay(700);

    //lining up for 2nd triple stack
    moveto(&mc, 36,33,30,3,10,-1);                                  //John: maybe lower error theta to avoid side of the robot contact disks
    mc.waitPosTime(3000);

    //second triple stack
    intake(&mc);
    moveto(&mc, 72,33,30,3); 
    intakeWaitForDiscs(&mc,3000);
    if(sensing.robot.magFullness != 3){
        moveto(&mc, 60,36,100,5,10,-1);                                  //John: maybe lower error theta to avoid side of the robot contact disks
        mc.waitPosTime(3000);

        moveto(&mc, 72,36,30,3,5);                                  //John: maybe lower error theta to avoid side of the robot contact disks=
        intakeWaitForDiscs(&mc,3000);
    }
    
    //shoot second triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc);
    sensing.robot.turretLock = true;
    delay(300);

    //collecting first of row of discs
    intake(&mc);
    moveto(&mc, 84,60,60,3);                                  //John: maybe lower error theta to avoid side of the robot contact disks
    mc.waitPosTime(4000);
    
    intake(&mc);
    moveto(&mc, 108,84,30,3, 5);                                  //John: maybe lower error theta to avoid side of the robot contact disks             //fixed?
    mc.waitPosTime(8000);
    
    //shoot second triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc, 1);
    sensing.robot.turretLock = true;
    delay(300);

    sensing.goal.xpos = 20;
    sensing.goal.ypos = 124;

    //picking up last 3 stack
    intake(&mc);
    moveto(&mc, 108,114,30,3, 5);
    mc.waitPosTime(9000);                                     //John: too long of wait time             //fixed?
    
    //shooting last triple stack
    movevoltage(&mc, 0,0);
    delay(400);
    shootdisks(&mc);
    delay(300);

    sensing.robot.turretLock = true;

    //move to roller
    moveto(&mc, 108, 150);
    startTime = millis();
    while(sensing.robot.ypos < 122 && millis() - startTime){
        delay(10);
    }
    rotateto(&mc, 90);
    waitRotate(&mc, 750);
    intake(&mc);

    mc.driveToRoller(2500);
    
    //move out of roller
    moveto(&mc, 144-37, 144-16,100,5,20,-1);
    while(sensing.robot.ypos>144-16){
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 144-48, 144-23,100,5,10,-1);
    mc.waitPosTime(2500);
    //line up with disk and roller might need a hold on drive motor
    rotateto(&mc, 0);
    startTime = millis();
    waitRotate(&mc, 2000);

    //pick up disk
    moveto(&mc, 144-0, 144-24,60,10,10,1);
    intake(&mc);
    startTime = millis();
    while(sensing.robot.xpos<144-24 &&millis()-startTime<3000){
        delay(10);
    }

    //shoot any in bot
    movevoltage(&mc, 0,0);
    delay(400);
    shootdisks(&mc);
    delay(300);

    //drive to roller
    moveto(&mc, 144-0, 144-24,100,10,5,1);
    startTime = millis();
    while((sensing.robot.xpos>pickPos(18, 0) || sensing.robot.xpos<pickPos(18, 1))&&millis()-startTime<1000){
        delay(10);
    }
    //get the roller
    intake(&mc);
    mc.driveToRoller(  2500);

    //shoot any in bot
    movevoltage(&mc, 0,0);
    delay(400);
    shootdisks(&mc);
    delay(300);
    sensing.goal.xpos = 72;
    sensing.goal.ypos = 72;

    sensing.robot.turretLock = false;
    movevoltage(&mc, 0,0);
    delay(500);
    mc.explode();

    while(1){
        delay(200);
    }

}