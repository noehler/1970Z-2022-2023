#include "main.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

void skillsAutonomous(void){
    motorControl_t mc;
    chaIntAng = 180;
    int revDist = 12;
    mc.rotateTo(180);
    mc.driveToRoller();
    mc.driveDist(-revDist);
    delay(300);
    mc.rotateTo(90);
    delay(300);
    mc.driveToRoller();
    //mc.driveDist(revDist);
    mc.driveDist(-revDist);
    delay(300);
    mc.rotateTo(135);
    delay(300);
    mc.driveDist(-90);
    delay(300);
    mc.rotateTo(0);
    mc.driveToRoller();
    //mc.driveDist(revDist);
    mc.driveDist(-revDist);
    delay(300);
    mc.rotateTo(-90);
    delay(300);
    mc.driveToRoller();
    //mc.driveDist(revDist);
    mc.driveDist(-revDist);
    delay(300);
    mc.rotateTo(315);
    mc.explode();

}