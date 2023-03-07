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
        delay(10);
    }
    //start aiming and and getting position for next disk
    sensing.robot.turretLock = false;
    setMotion(&mc, 25,49,100,5,80,-1);
    startTime = millis();
    while(sensing.robot.ypos<40&&millis()-startTime<500){
        delay(10);
    }
    setMotion(&mc, 25,49,100,5,10,-1);
    startTime = millis();
    mc.discCountChoice = 2;
    //finish moving and prep for shooting
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        delay(10);
    }
    //shoot
    startTime = millis();
    while(millis() - startTime <2000){
        if (fabs(mc.angdiff)>3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2))/2 +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
        }
    }
    mc.raiseAScore(3);//clear magazine shoot
    
    //line up with next three disks and reset turretangle pos
    mc.driveType = 1;
    mc.HeadingTarget = 45;
    mc.move.errtheta = 4;
    startTime = millis();
    sensing.robot.turretLock = true;
    while ((fabs(sensing.robot.angle-45)>mc.move.errtheta && millis() - startTime <1000)){
        delay(10);
    }
    //move on to next three disk and pick them up
    setMotion(&mc, 72,96,40,5,10,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while(distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance&&millis()-startTime<5000){
        delay(10);
    }
    //shoot three
    delay(500);
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    delay(30);
    mc.discCountChoice = 2;
    startTime = millis();
    while(millis() - startTime <3000){
        if (fabs(mc.angdiff)>2 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2))/2 < 30){
            break;
        }
    }
    mc.raiseAScore(3);//clear magazine shoot
    //moving on to next three stack
    sensing.robot.turretLock = true;
    mc.intakeRunning = 1;
    setMotion(&mc, 89,114,100,2,10,1);
    startTime = millis();
    while(distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance&&millis()-startTime<5000){
        delay(10);
    }
    setMotion(&mc, 84,109,40,1,10,-1);
    startTime = millis();
    while(distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance&&millis()-startTime<5000){
        delay(10);
    }
    setMotion(&mc, 89,114,40,2,10,1);
    startTime = millis();
    while(distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance&&millis()-startTime<5000){
        delay(10);
    }
    //line up for shoot
    delay(500);
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    mc.discCountChoice = 2;
    startTime = millis();
    while(millis() - startTime <3000){
        if (fabs(mc.angdiff)>3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2))/2 < 30){
            break;
        }
    }
    mc.raiseAScore(3);//clear magazine shoot
    //move to roller
    sensing.robot.turretLock = true;
    setMotion(&mc, 114,126,100,2,10,1);
    startTime = millis();
    while(distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance&&millis()-startTime<5000){
        delay(10);
    }
    mc.driveType = 1;
    mc.HeadingTarget = 90;
    startTime = millis();
    while ((fabs(sensing.robot.angle-45)>mc.move.errtheta && millis() - startTime <3000)){
        delay(10);
    }
    setMotion(&mc, 114,150,100,2,10,1);
    startTime = millis();
    while(sensing.robot.ypos<130){
        delay(10);
    }
    mc.driveToRoller(2000);
    while(1){
        delay(200);
    }

    //na te dr ove o ff the ro ad 
}