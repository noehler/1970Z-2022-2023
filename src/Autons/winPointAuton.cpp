#include "main.h"

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
    moveto(&mc, 48, 23,100,5,10,-1);
    mc.waitPosTime(2500,overallStartTime);

    //shooting
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime, 1);
    
    //picking up disc
    intake(&mc);
    moveto(&mc, 72, 48,40);
    intakeWaitForDiscs(&mc, 6000,overallStartTime);

    //shooting
    movevoltage(&mc, 0, 0);
    delay(200);
    shootdisks(&mc, overallStartTime,1);

    //move to lineup roller
    intake(&mc);
    moveto(&mc, 96, 60,100);
    mc.waitPosTime(2000,overallStartTime);
    intake(&mc);
    moveto(&mc, 126, 108,100);
    mc.waitPosTime(3000,overallStartTime);

    //rotating to face with roller
    rotateto(&mc, 0);
    waitRotate(&mc, 2000, overallStartTime);

    //getting final roller
    mc.driveToRoller(10000);
    
    movevoltage(&mc, 0, 0);
    while(1){

    }
}
