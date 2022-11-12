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

int odoCount = 0;
double sT;
void updateInfoLoop(void){
    //while (pros::competition::get_status() && COMPETITION_CONNECTED == true && COMPETITION_DISABLED == false){
    while (1){
        odometry();
        turAngupdate();
        //visionOdom();
        /*if (fabs ( robot.xposodom - robot.xposvision )< 5){
            robot.xpos = robot.xposodom;
        } else {
            robot.xpos = robot.xposvision;
            robot.xposodom = robot.xposvision;
        }
        if (fabs ( robot.yposodom - robot.yposvision )< 5){
            robot.xpos = robot.yposodom;
        } else {
            robot.xpos = robot.yposvision;
            robot.yposodom = robot.yposvision;
        }*/
        robotGoal.dx = homeGoal.xpos-robot.xpos;
        robotGoal.dy = homeGoal.ypos-robot.ypos;
        robotGoal.dz = homeGoal.zpos-robot.zpos;
        static int prevTime = millis();
		loopTimes[2] = millis() - prevTime;
		prevTime = millis();
        delay(10);
    }
}

void startLoop(void)
{

    while (1){
        //std::cout << "\nfps: " << odoCount /float((millis()-sT)/1000);
        singSameOldSongTimeTurretTwister();
        static int prevTime = millis();
		loopTimes[3] = millis() - prevTime;
		prevTime = millis();
        delay(20);
    }

}
