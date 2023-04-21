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

	sensing.set_status(37,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);

	//first roller
    mc.driveToRoller(4000, 2,1);

	//get first disc
	intake(&mc);
	moveto(&mc,24,24,100,5,5,1);
	waitPosTime(&mc, 1000, overallStartTime);

	//shoot all discs
	rotateto(&mc, goalAngle);
	delay(500);
	for (int i = 0;i <3;i++){
		shootdisks(&mc, overallStartTime,1500,1);
		delay(30);
	}


	//getting second roller
	moveto(&mc,11.5,37,100,3,5,-1);
	waitPosTime(&mc, 3000, overallStartTime);
	rotateto(&mc, 0);
	waitRotate(&mc, 1500, overallStartTime);
    mc.driveToRoller(4000, 2,2);

	//getting row
	intake(&mc);
	moveto(&mc,36,60,100,5,10,1);
	waitPosTime(&mc, 3000, overallStartTime);
	moveto(&mc,60,84,100,5,10,1);
	waitPosTime(&mc, 3000, overallStartTime);

	//shoot all discs
	rotateto(&mc, goalAngle);
	delay(500);
	for (int i = 0;i <3;i++){
		shootdisks(&mc, overallStartTime,1500,1);
		delay(30);
	}

	//getting triple stack
	mc.raise_intake.set_value(true);
	intake(&mc);
	moveto(&mc,84,108,100,3,5,1);
	waitPosTime(&mc, 3000, overallStartTime);
	mc.raise_intake.set_value(false);

	//finish picking up and go to point
	moveto(&mc,107,107,100,5,10,1);
	waitPosTime(&mc, 3000, overallStartTime);
	
	//shoot all discs
	rotateto(&mc, goalAngle);
	delay(700);
	for (int i = 0;i <3;i++){
		shootdisks(&mc, overallStartTime,1500,1);
		delay(30);
	}

	//getting roller
	moveto(&mc,133,107,100,3,10,-1);
	waitPosTime(&mc, 3000, overallStartTime);
	rotateto(&mc, 180);
	waitRotate(&mc, 1500, overallStartTime);
    mc.driveToRoller(4000, 2,2);
	
	//leave Roller
	moveto(&mc,107,107,100,3,5,1);
	waitPosTime(&mc, 3000, overallStartTime);
	
	//getting roller
	moveto(&mc,107,133,100,3,5,-1);
	waitPosTime(&mc, 3000, overallStartTime);
	rotateto(&mc, 270);
	waitRotate(&mc, 1500, overallStartTime);
    mc.driveToRoller(4000, 2,1);

	//leave Roller
	moveto(&mc,107,107,100,3,5,1);
	waitPosTime(&mc, 500, overallStartTime);

	//expand
	rotateto(&mc, atan2(72 - sensing.robot.ypos, 72 - sensing.robot.xpos));
	waitRotate(&mc, 2000, overallStartTime);
	mc.explode();
	
}