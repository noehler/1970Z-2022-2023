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
    while (runLoop){
        odometry();
        delay(20);
    }  
}

void startLoop(void)
{
    
    homeGoal.xpos = 20;
    homeGoal.ypos = 20;
    homeGoal.zpos = 25;
    
    robot.xpos = 0;
    robot.ypos = 0;
    robot.zpos = 14.4;

    robot.xVelocity = 0;
    robot.yVelocity = 0;

    Task turrC(turretControl);
    Task varUP(updateInfoLoop);

    while (runLoop){
        robotGoal.dx = homeGoal.xpos-robot.xpos;
        robotGoal.dy = homeGoal.ypos-robot.ypos;
        robotGoal.dz = -(robot.zpos-homeGoal.zpos);

        singSameOldSongTimeTurretTwister();
        std::cout << "\nX: " << robot.xpos << " Y: " << robot.ypos << " RR: " << radRotation;

        delay(20);
        
    }
    
}