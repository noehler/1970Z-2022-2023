#include "autonSetup.h"
#include "main.h"
#include "winPointAuton.h"

void winPointAuton(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

	sensing.set_status(37,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(2500, 2,1);

    //move to lineup disk
    moveto(&mc, 48, 23);
    waitPosTime(&mc,1000,overallStartTime);

    shootdisks(&mc, overallStartTime,1500,1);
    delay(200);
    shootdisks(&mc, overallStartTime,1500,1);

    //move to lineup roller
    intake(&mc);
    moveto(&mc, 128, 110,100,3,8);
    intakeWaitForDiscs(&mc, 5000,overallStartTime);
    
    //rotating to face with roller
    mc.intakeRunning = 0;
    rotateto(&mc, 180);
    waitRotate(&mc, 1500, overallStartTime);

    //getting final roller
    mc.driveToRoller(3000, 1,1);
    
    shootdisks(&mc, overallStartTime,1500,1);
    delay(20);
    movevoltage(&mc, 0, 0);
}

void closeWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

	sensing.set_status(37,11.5,90,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(2500, 2,1);

    //move to lineup disk
    moveto(&mc, 48, 23);
    waitPosTime(&mc,1000,overallStartTime);

    shootdisks(&mc, overallStartTime,1500,1);
    delay(200);
    shootdisks(&mc, overallStartTime,1500,1);
    
    mc.raise_intake.set_value(true);
    intake(&mc);
    moveto(&mc,44,20,100,2,5,-1);
    waitPosTime(&mc,1000,overallStartTime, 11000);
    mc.raise_intake.set_value(false);
    rotateto(&mc, 45);
    waitRotate(&mc, 2000, overallStartTime);
    if (mc.angdiff < 5){
        movevoltage(&mc, 5000, 5000);
        delay(3000);
        mc.raise_intake.set_value(false);
    }
    else{
        moveto(&mc,60,48);
        mc.raise_intake.set_value(false);
        intakeWaitForDiscs(&mc, 1000, overallStartTime);
    }
    shootdisks(&mc, overallStartTime,3000,1,0,13000);
    delay(200);
    shootdisks(&mc, overallStartTime,3000,1,0,14000);

    delay(20);
    movevoltage(&mc, 0, 0);
    
}

void farWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

    
}