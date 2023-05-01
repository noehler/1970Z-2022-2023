#include "autonSetup.h"
#include "main.h"
#include "robotConfig.h"

void skillsAutonomous(void){
	int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

	sensing.set_status(36,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(2000, 2,1);

    //move to lineup disk
    moveto(&mc, 48, 23);
    waitPosTime(&mc,500,overallStartTime);

    shootdisks(&mc, overallStartTime,6000,1);
    delay(200);
    shootdisks(&mc, overallStartTime,6000,1);

    //move to lineup roller
    intake(&mc);
    moveto(&mc, 125, 110,100,3,8);
    waitPosTime(&mc, 7500,overallStartTime);
    
    //rotating to face with roller
    mc.intakeRunning = 0;
    rotateto(&mc, 180);
    waitRotate(&mc, 1500, overallStartTime);

    //getting far roller
    mc.driveToRoller(3000, 1,1);
    
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);
    movevoltage(&mc, 0, 0);

	//REVERSE CYCLE
    moveto(&mc, 108, 136,100,3,5,-1);
    waitPosTime(&mc,500,overallStartTime);
	rotateto(&mc, 180);
	waitRotate(&mc, 500, overallStartTime);
	mc.driveToRoller(2000, 2,1);

    //move to lineup last roller
    intake(&mc);
    moveto(&mc, 19, 34,100,3,8);
    waitPosTime(&mc, 9000,overallStartTime);
    
    //rotating to face with roller
    mc.intakeRunning = 0;
    rotateto(&mc, 0);
    waitRotate(&mc, 500, overallStartTime);

    //getting far roller
    mc.driveToRoller(2000, 1,1);
    
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);
    movevoltage(&mc, 0, 0);

	//expand
	rotateto(&mc, atan2(72 - sensing.robot.ypos, 72 - sensing.robot.xpos));
	waitRotate(&mc, 2000, overallStartTime);
	mc.explode();
	
}