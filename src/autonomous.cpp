#include "devFunctions.h"
#include "pros/rtos.h"
#include "robotConfig.h"

bool winAuto = true;
bool scoreAuto = false;
bool testAuto = false;

void raiseAScore(void){
    elevatePiston.set_value(true);
    delay(500);
    elevatePiston.set_value(false);
    delay(1000);
    shootPiston.set_value(true);
    delay(1000);
    shootPiston.set_value(false);
}

void autonomousReal() {
  int startTime = c::millis();
  if (testAuto){
    robot.xpos = 0;
    robot.ypos = 0;

    robot.chaIntAng = 90;
    robot.TurintAng = 270;

    move.moveToxpos = 0;
    move.moveToypos = 24;
    while(move.reset == false && c::millis()-startTime < 3000){
        delay(20);
    }
    move.moveToxpos = 0;
    move.moveToypos = 0;
    move.moveToforwardToggle = -1;
    startTime = c::millis();
    while(move.reset == false && c::millis()-startTime < 3000){
        delay(20);
    }
  }
	if (winAuto){
        robot.chaIntAng = 270;
        robot.TurintAng = 90;

        robot.xpos = 29.4;
        robot.ypos = 14;

        homeGoal.xpos = 18;
        homeGoal.ypos = 126;
        move.tolerance = 0.5;
        move.moveToxpos = 29.4;
        move.moveToypos = 5;

        move.speed_limit = 30;
        while(move.reset == false && c::millis()-startTime < 3000){
            delay(20);
        }
        shootPiston.set_value(true);
        move.speed_limit = 100;
        chassis.isSpinner = true;
        delay(3000);
        shootPiston.set_value(false);

        chassis.isSpinner = false;

        move.reset = true;
        move.moveToforwardToggle = -1;
        move.moveToxpos = 30;
        move.moveToypos = 17;
        elevatePiston.set_value(true);
        startTime = c::millis();
        while(move.reset == false && c::millis()-startTime < 3000){
            delay(20);
        }
        elevatePiston.set_value(true);
        raiseAScore();
        move.moveToforwardToggle = 1;
        chassis.intakeRunning = 1;
        move.moveToxpos = 65;
        move.moveToypos = 45;
        move.tolerance = 3;

        startTime = c::millis();
        move.speed_limit = 60;
        while(move.reset == false && c::millis()-startTime < 3500){
            delay(20);
        }
        chassis.intakeRunning = 0;
        delay(200);
        raiseAScore();


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
