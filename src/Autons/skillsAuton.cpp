#include "autonSetup.h"
#include "main.h"
#include "robotConfig.h"

void finishIntakeAndShoot(void *mc, int overallStartTime){
	rotateto(&mc, goalAngle);
	delay(500);
	for (int i = 3-sensing.robot.magFullness;i <3;i++){
		shootdisks(&mc, overallStartTime,1500,1);
		delay(30);
	}
}

void skillsAutonomous(void){
	int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

	sensing.set_status(37,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);

	//first roller
    mc.driveToRoller(2500, 2,1);

	//get first disc
	intake(&mc);
	moveto(&mc,24,24,100,5,5,1);
	waitPosTime(&mc, 1000, overallStartTime);

	//shoot all discs
	finishIntakeAndShoot(&mc, overallStartTime);

	//getting second roller
	moveto(&mc,11.5,37,100,3,5,-1);
	waitPosTime(&mc, 1500, overallStartTime);
    mc.driveToRoller(2500, 2,1);

	//getting row
	intake(&mc);
	moveto(&mc,36,60,100,3,5,1);
	waitPosTime(&mc, 1500, overallStartTime);
	moveto(&mc,60,84,100,3,5,1);
	waitPosTime(&mc, 1500, overallStartTime);

	//shoot all discs
	finishIntakeAndShoot(&mc, overallStartTime);

	//getting triple stack
	mc.raise_intake.set_value(true);
	intake(&mc);
	moveto(&mc,84,108,100,3,5,1);
	waitPosTime(&mc, 1500, overallStartTime);
	mc.raise_intake.set_value(false);

	//finish picking up and go to point
	moveto(&mc,107,107,100,3,5,1);
	waitPosTime(&mc, 1500, overallStartTime);
	
	//shoot all discs
	finishIntakeAndShoot(&mc, overallStartTime);

	//getting roller
	moveto(&mc,133,107,100,3,5,-1);
	waitPosTime(&mc, 1500, overallStartTime);
    mc.driveToRoller(2500, 2,1);
	
	//leave Roller
	moveto(&mc,107,107,100,3,5,1);
	waitPosTime(&mc, 0500, overallStartTime);
	
	//getting roller
	moveto(&mc,107,133,100,3,5,-1);
	waitPosTime(&mc, 1500, overallStartTime);
    mc.driveToRoller(2500, 2,1);

	//leave Roller
	moveto(&mc,107,107,100,3,5,1);
	waitPosTime(&mc, 0500, overallStartTime);

	//expand
	rotateto(&mc, atan2(72 - sensing.robot.ypos, 72 - sensing.robot.xpos));
	mc.explode();
	
}