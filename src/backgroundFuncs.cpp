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
  sT = millis();
  //while (pros::competition::get_status() && COMPETITION_CONNECTED == true && COMPETITION_DISABLED == false){
  while (1){
      odometry();
      odoCount++;
      delay(10);
  }
}

void startLoop(void)
{

    while (1){
        std::cout << "\nfps: " << odoCount /float((millis()-sT)/1000);
        singSameOldSongTimeTurretTwister();
        delay(20);

    }

}
