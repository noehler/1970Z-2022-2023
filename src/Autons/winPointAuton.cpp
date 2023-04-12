#include "autonSetup.h"
#include "main.h"
#include "winPointAuton.h"

void winPointAuton(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

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
    moveto(&mc, 128, 105,60, 5,50);
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
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    sensing.set_status(37,18,270,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(3000);
    
    //move out of roller
    moveto(&mc, 37, 16,100,0,20,-1);
    while(sensing.robot.ypos<16){/////////////////////////////////////////////////////////////////////////////////////no time limit bomb
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 48, 24,100,5,10,-1);
    waitPosTime(&mc,2500,overallStartTime);

    //shooting
    mc.autoAim = true;
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime,false,1,12000,6000);
    shootdisks(&mc, overallStartTime,false,1,12000,6000);
    mc.autoAim = false;
    
    //picking up disc
    intake(&mc);
    moveto(&mc, 60, 36,60);
    intakeWaitForDiscs(&mc, 4000,overallStartTime);
    if (sensing.robot.magFullness != 3){
        moveto(&mc, 54, 30,100,5,5,-1);
        waitPosTime(&mc,1000,overallStartTime);
        moveto(&mc, 72, 48,60);
        intakeWaitForDiscs(&mc, 3000,overallStartTime);
    }
    
    //shooting
    mc.autoAim = true;
    movevoltage(&mc, 0, 0);
    delay(1000);
    shootdisks(&mc, overallStartTime);
    mc.autoAim = false;

}

void farWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    sensing.set_status(128.5,55,180,100, color);
    movevoltage(&mc, 0, 0);
    delay(2000);
    shootdisks(&mc, overallStartTime, 0,4,10000,10000);

    intake(&mc);
    moveto(&mc,100, 55, 80);
    intakeWaitForDiscs(&mc, 3500, overallStartTime,3,7000);

    if(sensing.robot.magFullness != 0){
        movevoltage(&mc, 0, 0);
        shootdisks(&mc, overallStartTime, 0,4,10000,3000);
        delay(20);
    }

    moveto(&mc,128, 55, 100,5,5,-1);
    waitPosTime(&mc,4000, overallStartTime);

    intake(&mc);
    moveto(&mc,128, 108, 100);
    waitPosTime(&mc,4000, overallStartTime);

    rotateto(&mc, 0);
    waitRotate(&mc, 1000, overallStartTime);
    
    mc.driveToRoller(5000,false);

    movevoltage(&mc, 0, 0);
}