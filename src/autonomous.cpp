#include "autonomous.h"
#include "devFunctions.h"
#include "pros/rtos.h"
#include "robotConfig.h"

autonMode_t autonMode = flipShoot;
startPos_t startPos = far;

void raiseAScore(void){
    elevatePiston.set_value(true);
    delay(350);
    elevatePiston.set_value(false);
    delay(250);
    shootPiston.set_value(true);
    delay(750);
    shootPiston.set_value(false);
}

void autonomousReal() {

    chassis.driveTrain.running = true;
    int startTime = c::millis();
	if (autonMode == flipShoot){
        if (startPos == near){
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
        }
        else{
            move.reset = true;
            robot.chaIntAng = 270;
            robot.TurintAng = 90;

            robot.xpos = 125;
            robot.ypos = 90.5;
            move.moveToxpos = robot.xpos;
            move.moveToypos = robot.ypos;
            delay(1000);
            
            shootPiston.set_value(true);
            delay(750);
            shootPiston.set_value(false);
            delay(500);
            raiseAScore();

            move.moveToforwardToggle = -1;
            move.moveToxpos = 125;
            move.moveToypos = 114;
            move.tolerance = .5;

            startTime = c::millis();
            move.reset = false;
            while(move.reset == false && c::millis()-startTime < 3000){
                delay(20);
            }
            move.reset = true;
            
            chassis.isSpinner = true;

            move.moveToforwardToggle = 1;
            move.moveToxpos = 144;
            move.moveToypos = 114;

            startTime = c::millis();
            move.reset = false;
            while (move.reset == false && c::millis()-startTime < 3000){
                delay(20);
            }
            move.reset = true;
            delay(20);
            chassis.isSpinner = false;

            move.moveToforwardToggle = -1;
            move.moveToxpos = 125;
            move.moveToypos = 100;

            startTime = c::millis();
            move.reset = false;
            while (move.reset == false && c::millis()-startTime < 3000){

            }
        }
    }
}
