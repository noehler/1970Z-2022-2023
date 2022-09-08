// made on July 28, 2022 by Nathaniel Oehler

#include "flywheelCode.h"
#include "main.h"
#include "odometry.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robotConfig.h"

bool runLoop = true;


void turrDrive(void){
    //while (pros::competition::get_status() && COMPETITION_CONNECTED == true && COMPETITION_DISABLED == false){
    while (1){
        turretAngleTo();
        double angle1 = double(turretAngle.get_position())/100;
        double angleDiff = angle1*12/259-robotGoal.angleBetweenHorREL;
        //std::cout <<angleDiff << "\n";
        int speedrot;
        if (fabs(angleDiff) >20){
            speedrot=60;
        }
        else if (fabs(angleDiff) > 2){
            speedrot=30;
        }
        else {
            speedrot = 0;
        }

        if (angleDiff < 0){
            speedrot *=-1;
        }
        
        diff1.move(-speedrot);
        diff2.move(speedrot);
        //std::cout <<"donemove\n";
        delay(20);
        //std::cout <<"about to restart\n";
    }  
}

void mainLoop(void)
{
    //Task my_task(turrDrive);
    std::cout << "starting Loop \n";
    
    while (runLoop){
        
        //basicOdometry();
        //lcd::clear();
        //3pros::lcd::print(0, "X: %f", robot.xpos);
        //pros::lcd::print(1, "Y: %f", robot.ypos);
        //pros::lcd::print(2, "Z:");
        
        delay(20);
    }
}