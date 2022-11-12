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
    waitShoot();
}

void autonomousReal() {
    int startTime = c::millis();
	if (autonMode == flipShoot){
        if (startPos == near){
            chassis.intakeRunning = 0;
            robot.chaIntAng = 270;
            robot.TurintAng = 90;

            robot.xpos = 29.4;
            robot.ypos = 18;

            homeGoal.xpos = 18;
            homeGoal.ypos = 126;
            move.tolerance = 1;
            move.moveToxpos = 29.4;
            move.moveToypos = 11;
            
            chassis.driveTrain.running = false;
            delay(2000);
            waitShoot();
            delay(500);
            raiseAScore();
            chassis.driveTrain.running = true;

            move.speed_limit = 40;
            move.resetMoveTo = false;
            startTime = c::millis();
            while(move.resetMoveTo == false && c::millis()-startTime < 3000){
                delay(20);
            }
            
            chassis.isSpinner = true;
            delay(1000);
            chassis.isSpinner = false;

            move.moveToforwardToggle = -1;
            move.moveToxpos = 29.4;
            move.moveToypos = 18;
            move.speed_limit = 40;
            move.resetMoveTo = false;
            startTime = c::millis();
            while(move.resetMoveTo == false && c::millis()-startTime < 3000){
                delay(20);
            }
            move.resetMoveTo = true;
            delay(300);

            chassis.intakeRunning = 1;
            move.moveToforwardToggle = 1;
            move.moveToxpos = 48;
            move.moveToypos = 24;
            move.speed_limit = 40;
            move.resetMoveTo = false;
            startTime = c::millis();
            while(move.resetMoveTo == false && c::millis()-startTime < 3000){
                delay(20);
            }
            move.resetMoveTo = true;
            chassis.intakeRunning = 1;
            delay(500);
            raiseAScore();
            move.moveToxpos = 72;
            move.moveToypos = 48;
            move.speed_limit = 40;
            
            delay(1000);
            raiseAScore();

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
