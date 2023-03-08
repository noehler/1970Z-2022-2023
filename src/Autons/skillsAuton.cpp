#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"
 double distto (double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
 }


void setMotion(void *mc, double xTo, double yTo, double speedLimit = 100, double tolerance = 5, double errtheta = 10, int forward = 1){
    //moveto frame parameters
    ((motorControl_t*) mc)->driveType = 0;
    ((motorControl_t*) mc)->move.moveToxpos = xTo;
    ((motorControl_t*) mc)->move.moveToypos = yTo;
    ((motorControl_t*) mc)->move.speed_limit = speedLimit;
    ((motorControl_t*) mc)->move.tolerance = tolerance;
    ((motorControl_t*) mc)->move.errtheta = errtheta;
    ((motorControl_t*) mc)->move.moveToforwardToggle = forward;
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
    double startTime = millis();
    sensing.set_status(37,18,270,100, 0);
    delay(50);
    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
    sensing.robot.turretLock = true;
    //move to roller
    setMotion(&mc, 37,-10, 100,0,10,1);
    while (sensing.robot.ypos>16){
        delay(10);
    }
    mc.driveToRoller(2500);
    //move out of roller
    setMotion(&mc, 37, 40,100,5,20,-1);
    while(sensing.robot.ypos<18){
        delay(10);
    }
    //move to lineup disk
    setMotion(&mc, 43, 28,100,2,10,-1);
    startTime = millis();
    while(sensing.robot.ypos<27 &&millis() - startTime <1000){
        delay(10);
    }
    //line up with disk and roller might need a hold on drive motor
    mc.driveType = 1;
    mc.HeadingTarget = 180;
    while (fabs(sensing.robot.angle -180)+fabs(sensing.robot.velW)>=3){ ///////////////////////////////////////////////////////////////////////////// no time limit
        delay(10);
    }
    //pick up disk
    setMotion(&mc, 0, 27,35,10,5,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while(sensing.robot.xpos>24&&millis()-startTime<1000){
        delay(10);
    }
    //drive to roller
    setMotion(&mc, 0, 26,100,10,5,1);
    startTime = millis();
    while(sensing.robot.xpos>18&&millis()-startTime<1000){
        delay(10);
    }
    //get the roller
    mc.intakeRunning = 0;
    mc.driveToRoller(2500);

    //out of roller
    setMotion(&mc, 20,26,50,5,10,-1);
    startTime = millis();
    while(sensing.robot.xpos<18&&millis()-startTime<1000){
        //time out
        //robot pos check
        delay(10);
    }
    //start aiming and and getting position for next disk
    sensing.robot.turretLock = false;
    mc.discCountChoice = 2;
    setMotion(&mc, 24,49,100,5,90,-1);
    startTime = millis();
    while(sensing.robot.xpos<24&&millis()-startTime<1000){
        // robot pos check
        //time out
        delay(10);
    }
    setMotion(&mc, 9,89,100,5,10,-1);
    //finish moving and prep for shooting
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        //robot to target dist check
        //time out
        delay(10);
    }
    //shoot by condition
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2))/2 +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    mc.raiseAScore(3);//first batch
    delay(300);
    //reset turret, rotate to disks
    sensing.robot.turretLock = true;
    mc.driveType = 1;
    mc.HeadingTarget = 0;
    startTime = millis();
    while (fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3000 &&fabs(mc.angdiff)>3 ){
        //robot heading check
        //time out
        //turret angle check before intake 
        delay(10);
    }
    //start intaking 
    mc.intakeRunning = 1;
    setMotion(&mc, 37.4,89,40,5,10,1);
    startTime = millis();
    while(sensing.robot.xpos<37.4 && millis() - startTime <5000 && sensing.robot.magFullness ==3){
        // check magazine full
        // check position
        // time out
        delay(10);
    }
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//second batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);
    sensing.robot.turretLock = true;

    setMotion(&mc, 55.5,82,100,1,10,-1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000 && fabs(mc.angdiff)>3){
        //robot to target dist check
        //time out
        //turret angle check before intake 
        delay(10);
    }
    mc.intakeRunning = 1;
    mc.driveType = 1;
    mc.HeadingTarget = 90;
    while (fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <2000  ){
        //robot heading check
        //time out
        delay(10);
    }

    setMotion(&mc, 55.5,124,40,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <5000&&sensing.robot.magFullness ==3){
        //robot to target dist check
        //time out
        //check if magazine if full
        delay(10);
    }
    
    mc.driveType = 3;
    mc.rightSpd = 0;
    mc.leftSpd = 0;
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//third batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);

    sensing.robot.turretLock = true;
    setMotion(&mc, 55.5,150,100,1,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while (sensing.robot.ypos<130 && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    
    mc.driveType = 1;
    mc.HeadingTarget = 320;
    startTime = millis();
    while (fabs(sensing.robot.angle -320)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3500){
        //time out
        //heading check
        delay(10);
    }
    setMotion(&mc, 84,108,100,1,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while (distto(sensing.robot.xpos-70,sensing.robot.ypos-117)>=5 && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    setMotion(&mc, 84,108,40,1,10,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while (distto(sensing.robot.xpos-84,sensing.robot.ypos-108)>=5 && millis() - startTime <2000&&sensing.robot.magFullness ==3){
        //robot to target dist check
        //time out
        delay(10);
    }
    setMotion(&mc, 70,117,100,1,10,-1);
    startTime = millis();
    while (distto(sensing.robot.xpos-70,sensing.robot.ypos-117)>=5 && millis() - startTime <2000 &&sensing.robot.magFullness ==3){
        //robot to target dist check
        //time out
        delay(10);
    }
    setMotion(&mc, 84,108,40,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-84,sensing.robot.ypos-108)>=5 && millis() - startTime <2000&&sensing.robot.magFullness ==3){
        //robot to target dist check
        //time out
        delay(10);
    }
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//forth batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);
    mc.driveType = 1;
    mc.HeadingTarget = 45;
    while (fabs(sensing.robot.angle -45)+fabs(sensing.robot.velW)>=3 &&millis() - startTime < 1000){
        //time out
        //heading check
        delay(10);
    }
    setMotion(&mc, 107,125,100,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-84,sensing.robot.ypos-108)>=2 && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    mc.driveType = 1;
    mc.HeadingTarget = 90;
    while (fabs(sensing.robot.angle -90)+fabs(sensing.robot.velW)>=3 &&millis() - startTime < 2000){
        //time out
        //heading check
        delay(10);
    }
    setMotion(&mc, 107,150, 100,0,10,1);
    while (sensing.robot.ypos<128){
        delay(10);
    }
    mc.driveToRoller(2500);
    while(1){
        delay(200);
    }
}