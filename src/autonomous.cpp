#include "devFunctions.h"
#include "pros/rtos.h"
#include "robotConfig.h"

bool isNear = true;

void autonomousReal() {
    int startTime = c::millis();
	if (1){
        shootPiston.set_value(true);
        robot.chaIntAng = 270;
        robot.TurintAng = 90;

        robot.xpos = 30;
        robot.ypos = 14;

        homeGoal.xpos = 20;
        homeGoal.ypos = 124;
        move.tolerance = 0.5;
        move.moveToxpos = 30;
        move.moveToypos = 13;

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
        move.moveToypos = 15;
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
        move.moveToxpos =48;
        move.moveToypos = 24;
        move.moveToforwardToggle = 1;
        move.tolerance = 3;
    } else{
        
    }
}