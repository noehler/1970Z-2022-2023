#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"
 double distto (double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
 }


void moveto(void *mc, double xTo, double yTo, double speedLimit = 100, double tolerance = 5, double errtheta = 10, int forward = 1){
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
void shootdisks(void *mc, int number = 4){
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
    ((motorControl_t*) mc)->updatedAD = false;
    while(millis() - starttime <5000){
        //time out
        if (((motorControl_t*) mc)->updatedAD && fabs(((motorControl_t*) mc)->angdiff)<3 && (fabs(((motorControl_t*) mc)->diffFlyWheelW) + fabs(((motorControl_t*) mc)->diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            std::cout<<"good turret"<<"\n";
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
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
void rotateto(void *mc,double ang){
    
    ((motorControl_t*) mc)->driveType = 1;
    ((motorControl_t*) mc)->HeadingTarget = ang;
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


void skillsAutonomous(void){
    
    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
    

    double startTime = millis();
    sensing.goalSpeed = 200;
    sensing.set_status(37,18,270,100, 0);
    sensing.goal.xpos = 124;
    sensing.goal.ypos = 20;
    sensing.robot.turretLock = true;
    delay(50);
    //move to roller
    mc.driveToRoller(2500);
    
    //move out of roller
    moveto(&mc, 37, 16,100,5,20,-1);
    while(sensing.robot.ypos<16){
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 48, 23,100,2,10,-1);
    startTime = millis();
    while(sensing.robot.ypos<27 &&millis() - startTime <4000){
        delay(10);
    }
    //line up with disk and roller might need a hold on drive motor
    rotateto(&mc, 180);
    startTime = millis();
    while(fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3000){
        delay(10);
    }

    //pick up disk
    moveto(&mc, 0, 26,35,10,10,1);
    intake(&mc);
    startTime = millis();
    while(sensing.robot.xpos>24 &&millis()-startTime<3000){
        delay(10);
    }

    //shooting batch 1
    movevoltage(&mc, 0,0);
    mc.intakeRunning = 0;
    sensing.robot.magFullness = 1;
    delay(400);
    shootdisks(&mc,1);
    delay(400);
    shootdisks(&mc,1);
    delay(400);
    shootdisks(&mc,1);
    
    //drive to roller
    moveto(&mc, 0, 26,100,10,5,1);
    startTime = millis();
    while((sensing.robot.xpos>pickPos(18, 0) || sensing.robot.xpos<pickPos(18, 1))&&millis()-startTime<1000){
        delay(10);
    }
    //get the roller
    mc.intakeRunning = 0;
    mc.driveToRoller(  2500);

    //out of roller
    moveto(&mc, 20,26,50,5,10,-1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(18, 0) || sensing.robot.xpos>pickPos(18, 1))&&millis()-startTime<1000){
        //time out
        //robot pos check
        delay(10);
    }

    //first triple stack
    intake(&mc);
    moveto(&mc, 48,40,30,3);
    mc.waitPosTime(10000);

    //shoot first triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc);
    sensing.robot.turretLock = true;
    delay(300);


    //second triple stack
    intake(&mc);
    moveto(&mc, 72,36,30,3);
    mc.waitPosTime(10000);

    //shoot second triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc);
    sensing.robot.turretLock = true;
    delay(300);

    movevoltage(&mc, 0,0);
    while(1){
        delay(200);
    }

}