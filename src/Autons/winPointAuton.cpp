#include "autonSetup.h"
#include "main.h"
#include "winPointAuton.h"

void winPointAuton(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    sensing.set_status(37,18,270,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(2500);
    
    //move out of roller
    moveto(&mc, 37, 16,100,0,20,-1);
    while(sensing.robot.ypos<16){/////////////////////////////////////////////////////////////////////////////////////no time limit bomb
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 48, 23,100,5,90,-1);
    mc.waitPosTime(1000,overallStartTime);

    //shooting
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime);
    
    //picking up disc
    intake(&mc);
    moveto(&mc, 72, 48,40);
    intakeWaitForDiscs(&mc, 4000,overallStartTime);

    //shooting
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime);

    //move to lineup roller
    intake(&mc);
    moveto(&mc, 96, 60,100);
    mc.waitPosTime(1000,overallStartTime);
    intake(&mc);
    moveto(&mc, 128, 105,100, 5,50);
    mc.waitPosTime(3000,overallStartTime);

    //rotating to face with roller
    rotateto(&mc, 0);
    waitRotate(&mc, 1500, overallStartTime);

    //getting final roller
    mc.driveToRoller(10000);
    
    movevoltage(&mc, 0, 0);
    while(1){

    }
}

void closeWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    sensing.set_status(37,18,270,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    mc.driveToRoller(2500);
    
    //move out of roller
    moveto(&mc, 37, 16,100,0,20,-1);
    while(sensing.robot.ypos<16){/////////////////////////////////////////////////////////////////////////////////////no time limit bomb
        delay(10);
    }
    
    //move to lineup disk
    moveto(&mc, 48, 16,100,5,10,-1);
    mc.waitPosTime(2500,overallStartTime);

    //shooting
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime);
    
    //picking up disc
    intake(&mc);
    moveto(&mc, 96, 12,50);
    intakeWaitForDiscs(&mc, 10000,overallStartTime);
    if (sensing.robot.magFullness != 3){
        moveto(&mc, 70, 12,100,5,-1);
        mc.waitPosTime(10000,overallStartTime);
        moveto(&mc, 96, 12,50);
        intakeWaitForDiscs(&mc, 10000,overallStartTime);
    }
    
    //shooting
    movevoltage(&mc, 0, 0);
    delay(1000);
    shootdisks(&mc, overallStartTime);

    while(1){
        delay(2000);
    }

}

void farWinPoint(void){
    int overallStartTime = millis();

    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");

    sensing.set_status(128.5,54.5,180,100, color);
    movevoltage(&mc, 0, 0);
    delay(20);
    shootdisks(&mc, overallStartTime, 0,4,10000,3000);

    intake(&mc);
    moveto(&mc,100, 54.5, 100);
    intakeWaitForDiscs(&mc, 7000, overallStartTime,3,9000);

    movevoltage(&mc, 0, 0);
    shootdisks(&mc, overallStartTime, 0,4,10000,3000);
    delay(20);

    moveto(&mc,128, 54.5, 100,5,5,-1);
    mc.waitPosTime(4000, overallStartTime);

    intake(&mc);
    moveto(&mc,128, 108, 100);
    mc.waitPosTime(4000, overallStartTime);

    rotateto(&mc, 0);
    waitRotate(&mc, 1500, overallStartTime);

    mc.driveToRoller(5000,false);

    movevoltage(&mc, 0, 0);
    while(1){
        delay(2000);
    }
}