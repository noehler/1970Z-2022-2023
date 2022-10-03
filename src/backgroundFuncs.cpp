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
    
    homeGoal.xpos = 120;
    homeGoal.ypos = 120;
    homeGoal.zpos = 25;
    
    robot.xpos = 72;
    robot.ypos = 72;
    robot.zpos = 12.2;

    robot.xVelocity = 0;
    robot.yVelocity = 0;

    double initialTurrAngle = 180;
    
    inertialTurret.set_rotation(initialTurrAngle);
    turretAngle.set_position(initialTurrAngle *100*259/12);

    Task turrC(turretControl);
    Task varUP(updateInfoLoop);

    while (runLoop){
        robotGoal.dx = homeGoal.xpos-robot.xpos;
        robotGoal.dy = homeGoal.ypos-robot.ypos;
        robotGoal.dz = -(robot.zpos-homeGoal.zpos);



        singSameOldSongTimeTurretTwister();

        delay(20);
        
    }
    
}