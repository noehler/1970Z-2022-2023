#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

void skillsAutonomous(void){
    motorControl_t mc;
    chaIntAng = 270;

    //starting controller threads
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    //first roller
    mc.move.moveToxpos = 30;
    mc.move.moveToypos = 12;
    mc.waitPosTime(4000);

    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 30;
    mc.move.moveToypos = 30;
    mc.waitPosTime(4000);

    //second roller
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 12;
    mc.move.moveToypos = 30;
    mc.waitPosTime(4000);

    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 30;
    mc.move.moveToypos = 30;
    mc.waitPosTime(4000);

    //first shot
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 48;
    mc.waitPosTime(4000);
    mc.raiseAScore(1);

    //collect discs
    mc.move.moveToxpos = 72;
    mc.move.moveToypos = 96;
    mc.intakeRunning = 1;
    mc.waitPosTime(4000);
    
    //second shot
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(1);

    //collect discs
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 96;
    mc.move.moveToypos = 120;
    mc.waitPosTime(4000);
    
    //third shot
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(1);

    //third roller
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 132;
    mc.waitPosTime(4000);

    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 114;
    mc.waitPosTime(4000);
    
    //fourth roller
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 132;
    mc.waitPosTime(4000);

    mc.move.moveToforwardToggle = -1;
    mc.move.moveToxpos = 114;
    mc.move.moveToypos = 114;
    mc.waitPosTime(4000);

    //collecting discs  
    mc.move.moveToforwardToggle = 1;
    mc.move.moveToxpos = 120;
    mc.move.moveToypos = 96;
    mc.waitPosTime(4000);
    
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 72;
    mc.move.moveToypos = 48;
    mc.waitPosTime(4000);

    //fourth shot
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(1);

    
    //collecting discs
    mc.intakeRunning = 1;
    mc.move.moveToxpos = 48;
    mc.move.moveToypos = 24;
    mc.waitPosTime(4000);

    //fifth shot
    delay(2000);
    mc.intakeRunning = 0;
    mc.raiseAScore(1);

    //expanding
    mc.move.moveToxpos = 24;
    mc.move.moveToypos = 24;
    
    sensing.goal.xpos = 72;
    sensing.goal.ypos = 124;
    sensing.goal.zpos = 72;  
    mc.waitPosTime(4000);

    mc.explode();

    while(1){
        delay(200);
    }

}