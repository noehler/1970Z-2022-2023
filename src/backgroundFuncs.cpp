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
        singSameOldSongTimeTurretTwister();
        robotGoal.angleBetweenHorREL = (inertial.get_heading() - robotGoal.angleBetweenHorABS);
    }  
}

void startLoop(void)
{
    //Task varUP(updateInfoLoop);
    homeGoal.xpos = 20;
    homeGoal.ypos = 20;
    homeGoal.zpos = 25;
    
    robot.xpos = 0;
    robot.ypos = 0;
    robot.zpos = 14.4;
    robot.xVelocity = 0;
    robot.yVelocity = 0;

    while (1){
        static bool oneRep = false;

        robot.xpos = getNum("\nRX: ");
        robot.ypos = getNum("\nRY: ");
        robot.xVelocity = getNum("\nVX: ");
        robot.yVelocity = getNum("\nVY: ");

        robotGoal.dx = homeGoal.xpos-robot.xpos;
        robotGoal.dy = homeGoal.ypos-robot.ypos;
        robotGoal.dz = -(robot.zpos-homeGoal.zpos);

        singSameOldSongTimeTurretTwister();

        if (!oneRep){
            Task turrC(turretControl);
            oneRep = true;
        }
        delay(20);
        
    }
    
}