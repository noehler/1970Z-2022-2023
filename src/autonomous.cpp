#include "autonomous.h"
#include "devFunctions.h"
#include "pros/rtos.h"
#include "robotConfig.h"

autonMode_t autonMode = flipShoot;
startPos_t startPos = far;
bool recoilPrevent = 0;

void raiseAScore(void){
    elevatePiston.set_value(true);
    delay(500);
    elevatePiston.set_value(false);
    delay(250);
    recoilPrevent = 1;
    shootPiston.set_value(true);
    delay(350);
    shootPiston.set_value(false);
    recoilPrevent = 0;
}

void autonomousReal() {
    int startTime = c::millis();
	if (autonMode == flipShoot){
        if (startPos == near){
            chassis.intakeRunning = 1;
            robot.chaIntAng = 270;
            robot.TurintAng = 90;

            robot.xpos = 29.4;
            robot.ypos = 15.75;

            homeGoal.xpos = 18;
            homeGoal.ypos = 126;
            move.tolerance = 2;
            move.moveToxpos = 29.4;
            move.moveToypos = 10;

            move.speed_limit = 40;
            move.resetMoveTo = false;
            while(move.resetMoveTo == false && c::millis()-startTime < 3000){
                delay(20);
            }
            chassis.isSpinner = true;
            delay(1500);
            shootPiston.set_value(true);
            recoilPrevent = 1;
            delay(500);
            shootPiston.set_value(false);
            recoilPrevent = 0;
            delay(500);
            raiseAScore();
            delay(500);
            raiseAScore();
            delay(500);
            raiseAScore();
            chassis.isSpinner = false;

        }
        else{
            robot.chaIntAng = 270;
            robot.TurintAng = 90;

            robot.xpos = 125;
            robot.ypos = 90.5;

            move.moveToforwardToggle = -1;
            move.moveToxpos = 125;
            move.moveToypos = 114;
            move.tolerance = 3;
            move.speed_limit = 100;

            startTime = c::millis();
            move.resetMoveTo = false;
            while(move.resetMoveTo == false && c::millis()-startTime < 3000){
                delay(20);
            }
            move.resetMoveTo = true;
            shootPiston.set_value(true);
            delay(750);
            shootPiston.set_value(false);
            delay(500);
            raiseAScore();
            
            chassis.isSpinner = true;

            move.moveToforwardToggle = 1;
            move.moveToxpos = 144;
            move.moveToypos = 114;
            move.speed_limit = 50;

            startTime = c::millis();
            move.resetMoveTo = false;
            while (move.resetMoveTo == false && c::millis()-startTime < 3000){
                delay(20);
            }
            move.resetMoveTo = true;
            delay(20);
            chassis.isSpinner = false;
        }
    }
}
