#include "main.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

void skillsAutonomous(void){
    motorControl_t mc;
    chaIntAng = 180;
    while (1){
        mc.rotateTo(90);
        delay(300);
        mc.rotateTo(0);
        delay(300);
        mc.rotateTo(-90);
        delay(300);
        mc.rotateTo(180);
        delay(300);
    }
    //shoot preload discs
    Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");

    sensing.goalSpeed = 310;
    goalAngle = 0;

    delay(5000);
    mc.raiseAScore(1);
    sensing.goalSpeed = 0;

    //setting up initial variables
    chaIntAng = 180;
    int revDist = 12;

    //first Roller
    mc.rotateTo(180);
    mc.driveToRoller();
    mc.driveDist(-revDist);
    delay(300);

    //secondRoller
    mc.rotateTo(90);
    delay(300);
    mc.driveToRoller();
    mc.driveDist(-revDist);
    delay(300);

    //moving to opposite corner
    mc.rotateTo(135);
    delay(300);
    mc.driveDist(-85);
    delay(300);

    //third Roller
    mc.rotateTo(0);
    mc.driveToRoller();
    mc.driveDist(-revDist);

    //last roller
    delay(300);
    mc.rotateTo(-90);
    delay(300);
    mc.driveToRoller();
    mc.driveDist(-revDist);
    delay(300);

    //expand
    mc.rotateTo(315);
    mc.explode();

    while(1){
        delay(200);
    }
}