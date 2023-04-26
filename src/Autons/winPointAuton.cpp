#include "autonSetup.h"
#include "main.h"
#include "robotConfig.h"
#include "winPointAuton.h"

void winPointAuton(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

	sensing.set_status(36,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(3000, 2,1);

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

    //getting final roller
    mc.driveToRoller(3000, 1,1);
    
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);
    movevoltage(&mc, 0, 0);
    delay(10);
}

void closeWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");
	
    sensing.set_status(36,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    shootdisks(&mc, overallStartTime,2000000,1);
    //mc.driveToRoller(2000, 2,1);
    
    //move to lineup disk
    moveto(&mc, 48, 23);
    waitPosTime(&mc,1000,overallStartTime);

    shootdisks(&mc, overallStartTime,6000,1);
    delay(500);
    shootdisks(&mc, overallStartTime,6000,1);
    
    mc.raise_intake.set_value(true);
    intake(&mc);
    moveto(&mc,60,36,100,6);
    while(sensing.robot.ypos <30){
        delay(20);
    }
    movevoltage(&mc, 4000, 4000);
    delay(900);
    mc.raise_intake.set_value(false);
    rotateto(&mc, goalAngle);
    delay(1500);
    shootdisks(&mc, overallStartTime,6000,1);
    delay(200);
    shootdisks(&mc, overallStartTime,6000,1);
    delay(200);
    shootdisks(&mc, overallStartTime,6000,1);

    delay(20);
    movevoltage(&mc, 1, 1);
    
}

void farWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

	sensing.set_status(129,91,270,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);

    //move to lineup roller
    moveto(&mc, 128, 110,100,2,8,-1);
    waitPosTime(&mc, 1500, overallStartTime);
    
    //rotating to face with roller
    rotateto(&mc, 180);
    waitRotate(&mc, 1500, overallStartTime);

    //getting far roller
    mc.driveToRoller(3000, 1,1);
    shootdisks(&mc, overallStartTime,6000,1);
    shootdisks(&mc, overallStartTime,6000,1);

    //getting discs
    intake(&mc);
    moveto(&mc, 84,60, 100,5,7);
    intakeWaitForDiscs(&mc, 5000, overallStartTime);
    rotateto(&mc,goalAngle);
    delay(1000);
    mc.intakeRunning = 0;
    shootdisks(&mc, overallStartTime,10000,1);
    shootdisks(&mc, overallStartTime,10000,1);
    shootdisks(&mc, overallStartTime,10000,1);
    
    delay(20);
    movevoltage(&mc, 0, 0);
}