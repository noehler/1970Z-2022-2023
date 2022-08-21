// made on July 28, 2022 by Nathaniel Oehler

#include "flywheelCode.h"
#include "main.h"
#include "robotConfig.h"

void mainLoop(void)
{
    while (1){
        basicOdometry();

        if (pros::competition::get_status() && COMPETITION_DISABLED == true ){
            turretSpeed();
            turretAngleTo();
        }
        
        delay(20);
    }
}