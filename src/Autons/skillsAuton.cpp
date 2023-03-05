#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

void skillsAutonomous(void){
    motorControl_t mc;
    chaIntAng = 270;
    sensing.robot.xpos = 36;
    sensing.robot.ypos = 12;

    //starting controller threads
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    //first roller
    mc.driveToRoller(5000);

    mc.move.errtheta = 10;
    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 42;
    mc.move.moveToypos = 24;
    mc.waitPosTime(2000);

    //second roller
    mc.move.moveToforwardToggle = 1;
    mc.intakeRunning = 1;
    mc.move.speed_limit = 50;
    mc.move.errtheta = 5;
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 24;
    mc.waitPosTime(4000);

    mc.move.moveToxpos = 18;
    mc.move.moveToypos = 30;
    mc.waitPosTime(6000);
    mc.intakeRunning = 0;

    mc.rotateTo(180);

    mc.move.errtheta = 10;
    mc.driveToRoller(5000);    
    mc.raiseAScore(3);
    mc.move.speed_limit = 127;
    mc.move.moveToforwardToggle = -1;
    
    mc.waitPosTime(10000);

    mc.move.tolerance = 5;

    //first shot
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 43;
    mc.waitPosTime(10000);
    mc.raiseAScore(3);
    delay(2000);

    //collect discs
    mc.move.moveToxpos = 36;
    mc.move.moveToypos = 52;
    mc.waitPosTime(4000);
    mc.move.moveToxpos = 48;
    mc.move.moveToypos = 64;
    mc.intakeRunning = 1;
    mc.waitPosTime(4000);
    mc.move.moveToxpos = 60;
    mc.move.moveToypos = 76;
    mc.intakeRunning = 1;
    mc.waitPosTime(4000);
    mc.move.moveToxpos = 72;
    mc.move.moveToypos = 96;
    mc.intakeRunning = 1;
    mc.waitPosTime(4000);
    
    //second shot
    mc.move.speed_limit = 127;
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(3);
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
    mc.raiseAScore(3);
    delay(2000);

    //third roller
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 132;
    mc.waitPosTime(10000);
    mc.rotateTo(90);
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
    mc.rotateTo(0);
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
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(3);
    delay(2000);
    
    //collecting discs
    mc.move.speed_limit = 30;
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 48;
    mc.move.moveToypos = 24;
    mc.waitPosTime(10000);
    mc.move.speed_limit = 127;

    //fifth shot
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(3);
    delay(2000);

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