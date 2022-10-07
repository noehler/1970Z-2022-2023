// made on July 28, 2022 by Nathaniel Oehler

#include "devFunctions.h"
#include "flywheelCode.h"
#include "main.h"
#include "odometry.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robotConfig.h"

bool runLoop = true;


void updateInfoLoop(void){
    //while (pros::competition::get_status() && COMPETITION_CONNECTED == true && COMPETITION_DISABLED == false){
    while (1){
        odometry();
        delay(20);
    }  
}

void startLoop(void)
{

    while (1){
        robotGoal.dx = homeGoal.xpos-robot.xpos;
        robotGoal.dy = homeGoal.ypos-robot.ypos;
        robotGoal.dz = homeGoal.zpos-robot.zpos;

        singSameOldSongTimeTurretTwister();

        delay(20);
        
    }
    
}