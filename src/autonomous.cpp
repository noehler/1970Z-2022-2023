#include "devFunctions.h"
#include "pros/rtos.h"
#include "robotConfig.h"

bool isNear = true;

void autonomousReal() {
    /*std::cout << "\nautoRunning";
    inertial.set_heading(270);
    inertialTurret.set_data_rate(90);
    robot.xpos = 0;
    robot.ypos = 0;

    move.moveToxpos = 0;
    move.moveToypos = 30;
    while (1){
        delay(20);
        std::cout << "\nautovalsUpdated";
    }*/
    std::cout << "\nautovalsUpdated";
    int startTime = c::millis();
	if (isNear){
        inertial.set_rotation(90);
        inertialTurret.set_rotation(270);
        shootPiston.set_value(true);
        robot.xpos = 30;
        robot.ypos = 13;
        homeGoal.xpos = 20;
        homeGoal.xpos = 124;
        move.moveToxpos = 30;
        move.moveToypos = 0;
        move.speed_limit = 30;
        while(move.reset == false && c::millis()-startTime < 3000){
            delay(20);
        }
        shootPiston.set_value(false);
        move.speed_limit = 100;
        move.reset = true;
        chassis.isSpinner = true;
        delay(5000);

        chassis.isSpinner = false;

        move.moveToforwardToggle = -1;
        move.moveToxpos = 30;
        move.moveToypos = 30;
        while(move.reset == false && c::millis()-startTime < 9000){
            delay(20);
        }
        elevatePiston.set_value(true);
        delay(100);
        elevatePiston.set_value(false);
        delay(100);
        elevatePiston.set_value(true);
        delay(100);
        elevatePiston.set_value(false);
        delay(100);
        elevatePiston.set_value(true);
        delay(100);
        elevatePiston.set_value(false);
        delay(100);
        shootPiston.set_value(true);
        delay(400);
        elevatePiston.set_value(false);
    } else{
        
    }
}