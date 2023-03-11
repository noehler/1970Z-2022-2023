#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

void skillsAutonomous(void){

    int overallStartTime = millis();
    
    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
    

    double startTime = millis();
    sensing.goalSpeed = 180;
    sensing.set_status(37,18,270,100, 1);
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
    mc.waitPosTime(2500,overallStartTime);
    //line up with disk and roller might need a hold on drive motor
    rotateto(&mc, 180);
    startTime = millis();
    waitRotate(&mc, 1000,overallStartTime);

    //pick up disk
    moveto(&mc, 0, 23,40,10,3,1);                                  //John: potentially too big of error heading, but at the same time, the pid spin is wack so idk.
    intake(&mc);
    startTime = millis();
    while(sensing.robot.xpos>24 &&millis()-startTime<5000){
        delay(10);
    }

    //shooting batch 1
    movevoltage(&mc, 0,0);
    mc.intakeRunning = 0;                                           //John: between each shoots, time can be lowered to 200ms or lower 
    delay(200);
    shootdisks(&mc,overallStartTime,0,1);
    delay(200);
    shootdisks(&mc,overallStartTime,0,1);
    delay(200);
    shootdisks(&mc,overallStartTime,0,1);
    
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
    
    intakeWaitForDiscs(&mc,5000,overallStartTime);
    if(sensing.robot.magFullness != 3){
        moveto(&mc, 40,33,100,5,10,-1);
        mc.waitPosTime(3000,overallStartTime);

        moveto(&mc, 48,40,30,3);
        intakeWaitForDiscs(&mc,3000,overallStartTime);
    }
    delay(600);

    //shoot first triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc,overallStartTime);
    sensing.robot.turretLock = true;
    delay(700);

    //lining up for 2nd triple stack
    moveto(&mc, 36,33,100,3,10,-1);                                  //John: maybe lower error theta to avoid side of the robot contact disks
    mc.waitPosTime(3000,overallStartTime);

    //second triple stack
    intake(&mc);
    moveto(&mc, 72,33,40,3); 
    intakeWaitForDiscs(&mc,3000,overallStartTime);
    if(sensing.robot.magFullness != 3){
        moveto(&mc, 60,36,100,5,10,-1);                                  //John: maybe lower error theta to avoid side of the robot contact disks
        mc.waitPosTime(3000,overallStartTime);

        moveto(&mc, 72,36,30,3,5);                                  //John: maybe lower error theta to avoid side of the robot contact disks=
        intakeWaitForDiscs(&mc,3000,overallStartTime);
    }
    
    //shoot second triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc,overallStartTime);
    sensing.robot.turretLock = true;
    delay(300);

    //moving back to line up with discs
    moveto(&mc, 72,33,100,3,10,-1); 
    intakeWaitForDiscs(&mc,2000,overallStartTime);

    //collecting first of row of discs
    intake(&mc);
    moveto(&mc, 84,60,60,3);                                  //John: maybe lower error theta to avoid side of the robot contact disks
    mc.waitPosTime(4000,overallStartTime);
    
    intake(&mc);
    moveto(&mc, 108,84,40,3, 5);                                  //John: maybe lower error theta to avoid side of the robot contact disks             //fixed?
    mc.waitPosTime(8000,overallStartTime);
    
    //shoot second triple stack
    movevoltage(&mc, 0,0);
    shootdisks(&mc,overallStartTime);
    sensing.robot.turretLock = true;
    delay(300);

    sensing.goal.xpos = 20;
    sensing.goal.ypos = 124;

    //picking up last 3 stack
    intake(&mc);
    moveto(&mc, 108,114,30,3, 5);
    mc.waitPosTime(9000,overallStartTime,1);                                     //John: too long of wait time             //fixed?
    
    //shooting last triple stack
    movevoltage(&mc, 0,0);
    delay(400);
    shootdisks(&mc,overallStartTime);
    delay(300);

    sensing.robot.turretLock = true;

    //move to roller
    moveto(&mc, 108, 150);
    startTime = millis();
    while(sensing.robot.ypos < 122 && millis() - startTime){
        delay(10);
    }
    rotateto(&mc, 90);
    waitRotate(&mc, 750,overallStartTime);
    intake(&mc);

    mc.driveToRoller(2500);
    
    //move out of roller
    moveto(&mc, 144-37, 144-18,100,5,20,-1);
    while(sensing.robot.ypos>144-18){
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 144-48, 144-23,100,5,10,-1);
    mc.waitPosTime(2500,overallStartTime);
    //line up with disk and roller might need a hold on drive motor
    rotateto(&mc, 0);
    startTime = millis();
    waitRotate(&mc, 2000,overallStartTime);

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
    shootdisks(&mc,overallStartTime);
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

    //expanding
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