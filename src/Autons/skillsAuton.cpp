#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"
 
void skillsAutonomous(void){
    sensing.set_status(36,18,270,100, 0);
    motorControl_t mc;
    double totalX = 0;
    double totalY = 0;
    int startTime = millis();
    int loops = 0;
    /*while (millis() - startTime < 1000){
        totalX+=sensing.robot.GPSxpos;
        totalY+=sensing.robot.GPSypos;
        loops++;
        delay(10);
    }*/
    //sensing.set_status(totalX/loops,totalY/loops,270,100, 0);
    mc.move.tolerance = 5;
    mc.move.errtheta = 10;
    sensing.robot.turretLock = true;

    //starting controller threads
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
    
    
    mc.driveType = 1;
    mc.HeadingTarget = 180; 










    delay(10000000);
    //first roller
    mc.move.moveToxpos = 36;
    mc.move.moveToypos = -10;
    while (sensing.robot.ypos >=16){
        delay(10);
    }
    mc.driveType = 1;
    mc.HeadingTarget = 270; 
    mc.rotateTo();
    while (fabs(sensing.robot.angle)-fabs(mc.HeadingTarget)>mc.move.errtheta){
        delay(10);
    }
    mc.driveType = 3;
    mc.driveToRoller(5000);
    
    
    mc.move.moveToxpos = 42;
    mc.move.moveToypos = 24;

    mc.move.errtheta = 10;
    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 42;
    mc.move.moveToypos = 24;
    mc.waitPosTime(10000);

    //collecting discs
    mc.move.errtheta = 10;
    mc.move.tolerance = 2;
    mc.move.moveToforwardToggle = 1;
    mc.intakeRunning = 1;
    mc.move.speed_limit = 50;
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 24;
    mc.waitPosTime(10000);

    mc.move.errtheta = 15;
    mc.move.moveToxpos = 18;
    mc.move.moveToypos = 32;
    mc.waitPosTime(10000);
    mc.intakeRunning = 0;

    //hitting roller
    mc.move.errtheta = 10;
    mc.move.tolerance = 5;
    mc.rotateTo();

    mc.driveToRoller(5000);  
    sensing.robot.turretLock = false;
    mc.raiseAScore(3);
    sensing.robot.turretLock = true;
    mc.move.speed_limit = 127;
    mc.move.moveToforwardToggle = -1;
    
    mc.waitPosTime(10000);

    //first shot
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 43;
    mc.waitPosTime(10000);
    sensing.robot.turretLock = false;
    mc.raiseAScore(3);
    sensing.robot.turretLock = true;
    delay(2000);

    //collect discs
    mc.move.speed_limit = 30;
    mc.move.errtheta = 5;
    mc.move.tolerance = 3;
    mc.move.moveToxpos = 72;
    mc.move.moveToypos = 96;
    mc.intakeRunning = 1;
    mc.waitPosTime(12000);
    
    //second shot
    mc.move.speed_limit = 127;
    delay(2000);
    sensing.robot.turretLock = false;
    mc.raiseAScore(3);
    sensing.robot.turretLock = true;
    delay(2000);

    //collect discs
    mc.move.tolerance = 3;
    mc.move.speed_limit = 30;
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 96;
    mc.move.moveToypos = 120;
    mc.waitPosTime(10000);
    mc.move.speed_limit = 127;
    
    //third shot
    delay(4000);
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    mc.raiseAScore(3);
    sensing.robot.turretLock = true;
    delay(1000);
    mc.move.tolerance = 5;
    mc.move.errtheta = 10;

    //third roller
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 132;
    mc.waitPosTime(10000);
    mc.rotateTo();
    mc.driveToRoller(4000);

    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 114;
    mc.waitPosTime(10000);
    
    //fourth roller
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 132;
    mc.waitPosTime(10000);
    mc.rotateTo();
    mc.driveToRoller(4000);


    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 114;
    mc.waitPosTime(10000);

    //collecting discs  
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 120;
    mc.move.moveToypos = 96;
    mc.waitPosTime(10000);
    
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 72;
    mc.move.moveToypos = 48;
    mc.waitPosTime(10000);

    //fourth shot
    delay(1000);
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    mc.raiseAScore(3);
    sensing.robot.turretLock = true;
    delay(2000);
    
    //collecting discs
    mc.move.speed_limit = 30;
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 48;
    mc.move.moveToypos = 24;
    mc.waitPosTime(10000);
    mc.move.speed_limit = 127;

    //fifth shot
    mc.intakeRunning = 0;
    delay(1000);
    sensing.robot.turretLock = false;
    mc.raiseAScore(3);
    sensing.robot.turretLock = true;
    delay(1000);

    //expanding
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 24;
    
    sensing.goal.xpos = 72;
    sensing.goal.ypos = 124;
    sensing.goal.zpos = 72;  
    mc.waitPosTime(10000);

    mc.explode();

    while(1){
        delay(200);
    }

}