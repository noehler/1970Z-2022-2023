// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"
#include "robotConfig.h"

void mainLoop(void)
{
    while (1){
        basicOdometry();

        if (pros::competition::get_status() && COMPETITION_ENABLED == true ){
            turretControl();
        }
        
        delay(20);
    }
}