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
    mc.driveToRoller(2500, 1, -1);
    
    //move out of roller
    moveto(&mc, 37, 16,100,0,20,1);
    while(sensing.robot.ypos<16){/////////////////////////////////////////////////////////////////////////////////////no time limit bomb
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 48, 23,100,5,90,-1);
    waitPosTime(&mc,1000,overallStartTime);

    //shooting
    mc.autoAim = true;
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime);
    mc.autoAim = false;

    //move to lineup roller
    intake(&mc);
    moveto(&mc, 128, 105,100, 5,50);
    intakeWaitForDiscs(&mc, 9000,overallStartTime);

    //shooting
    mc.autoAim = true;
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime);
    mc.autoAim = false;
    
    intake(&mc);
    moveto(&mc, 128, 104,100, 5,50);
    waitPosTime(&mc,3000, overallStartTime);

    //trying to clear disks
    mc.intakeRunning = 2;
    movevoltage(&mc, -12000, 12000);
    delay(500);
    
    //rotating to face with roller
    mc.intakeRunning = 0;
    rotateto(&mc, 0);
    waitRotate(&mc, 1500, overallStartTime);

    //getting final roller
    mc.driveToRoller(10000, 0);
    
    movevoltage(&mc, 0, 0);
}

void closeWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

    
}

void farWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
    Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");

    
}