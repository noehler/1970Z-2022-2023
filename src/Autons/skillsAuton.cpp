#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

void skillsAutonomous(void){
    motorControl_t mc;
    chaIntAng = 180;

    //starting controller threads
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    //getting discs from dump
    while(1){
        mc.move.moveToxpos = 72;
        mc.move.moveToypos = 5;
        mc.move.speed_limit = 50;
        mc.move.moveToforwardToggle = -1;


        mc.intakeRunning = 1;

        mc.waitPosTime(4000);

        mc.move.moveToxpos = 72;
        mc.move.moveToypos = 40;
        mc.move.speed_limit = 80;
        mc.move.moveToforwardToggle = -1;

        mc.intakeRunning = 0;
        delay(200);

        mc.raiseAScore(1);

        mc.waitPosTime(2000);
    }
    

}