#include "devFunctions.h"
#include "pros/rtos.h"
#include "robotConfig.h"

bool winAuto = false;
bool scoreAuto = true;

void raiseAScore(void){
    elevatePiston.set_value(true);
    delay(500);
    elevatePiston.set_value(false);
    delay(500);
    shootPiston.set_value(false);
    delay(1000);
    shootPiston.set_value(true);
}

void autonomousReal() {
    int startTime = c::millis();
	if (winAuto){
        robot.chaIntAng = 270;
        robot.TurintAng = 90;

        robot.xpos = 29.4;
        robot.ypos = 14;

        homeGoal.xpos = 20;
        homeGoal.ypos = 124;
        move.tolerance = 0.5;
        move.moveToxpos = 29.4;
        move.moveToypos = 13;

        move.speed_limit = 30;
        while(move.reset == false && c::millis()-startTime < 3000){
            delay(20);
        }
        shootPiston.set_value(false);
        move.speed_limit = 100;
        chassis.isSpinner = true;
        delay(5000);
        shootPiston.set_value(true);

        chassis.isSpinner = false;

        move.reset = true;
        move.moveToforwardToggle = -1;
        move.moveToxpos = 35;
        move.moveToypos = 14;
        while(move.reset == false && c::millis()-startTime < 9000){
            delay(20);
        }
        elevatePiston.set_value(true);
        delay(500);
        elevatePiston.set_value(false);
        delay(500);
        elevatePiston.set_value(true);
        delay(500);
        elevatePiston.set_value(false);
        delay(500);
        elevatePiston.set_value(true);
        delay(500);
        elevatePiston.set_value(false);
        delay(500);
        shootPiston.set_value(false);
        delay(5000);
        shootPiston.set_value(true);
        move.moveToforwardToggle = 1;
        move.tolerance = 3;
    } else if (scoreAuto){
        
        

        robot.xpos = 144-robot.width/2;
        robot.ypos = 72;
        robot.zpos = 12.2;
        move.moveToxpos = 144-robot.width/2;
        move.moveToypos = 72;

        delay(500);
        shootPiston.set_value(false);
        delay(1000);
        shootPiston.set_value(true);
        delay(500);
        raiseAScore();

        chassis.intakeRunning = 2;
        move.moveToxpos = 108;
        move.moveToypos = 85;

        while (move.reset == false && startTime <9000){

        }
        raiseAScore();

        
    }
}