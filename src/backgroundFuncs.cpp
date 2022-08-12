// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"
#include "robotConfig.h"

void mainLoop(void)
{
    while (1){
        basicOdometry();
        std::cout << "X: " << robot.xpos << "\nY: " << robot.ypos << "\nZ: " << robot.zpos << "\n\n";
        std::cout << "FB: " << leftEncoderFB.get_value() << "\nSS: " << encoderLR.get_value() << "\n\n";
        delay(40);
    }
}