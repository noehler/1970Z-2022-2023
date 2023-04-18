#include "main.h"

void skillsAutonomous(void){

    int overallStartTime = millis();
    
    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void*) &mc, "Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void*) &sensing, "turret angle Task");
    

}