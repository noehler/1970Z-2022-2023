#include "robotConfig.h"

void motorControl(void){
  while(runLoop){
    double turrAngle = -float(turretEncoder.get_position())/100/259*12;
    double turrAngleABS  = inertial.get_rotation() + turrAngle;

    static double diffInSpd;
    //std::cout << "\nTAngleRel: " << turrAngle;  
    bool dtb = false;


    std::cout << "\ninert: " << inertialTurret.get_rotation() << ", enc: " << -turrAngleABS;

    if (fabs(-turrAngleABS + inertialTurret.get_rotation()) > 3 && abs(turretEncoder.get_velocity()) < 300){
      turretEncoder.set_position((-inertialTurret.get_rotation() + inertial.get_rotation())*100*259/12);
      dtb = 1;
      std::cout << "\ndiff: " << turrAngleABS + inertialTurret.get_rotation() << ", new: " << -turrAngleABS;
    }
    else{
    }
    /*std::cout << "\ndiff: " << fabs(turrAngleABS + inertialTurret.get_rotation()) << ",     IturrAngle: " << inertialTurret.get_rotation() << ",    AbsRot: " << turrAngleABS
      << ",    IbaseRot: "<< inertial.get_rotation() << ",    vel: " << turretEncoder.get_velocity() << ",     diff2Big: " << dtb;*/

    double turrAngleBet = robotGoal.angleBetweenHorABS + turrAngleABS;

    if (turrAngleBet > 180){
      turrAngleBet -= 360;
    }
    else if( turrAngleBet < -180){
      turrAngleBet += 360;
    }

    int baseSPD;
    //feed forward code todo here, 
    //chassie rotation rate = chassie change of angle in last cycle / elapsed time of last cycle (odom loop)
    //turret target speed = turret ang dif / elapsed time of last cycle (turret twister loop)
    //turret target speed = -chassie rotation rate (because turret need to go to opposite of chassie rotation) + turret target speed
    //feed above calculated speed into a PID

    //slowing down turret when nothing is loaded or on deck so intake can run faster
    if (deckLoaded.get_value() > 1900){
      diffInSpd = pow(fabs(turrAngleBet), 1.4/3)*18; // put that PID here
    }
    else{
      diffInSpd = pow(fabs(turrAngleBet), 1.4/3)*5; // put that PID here
    }

    //---------------------this might be the reason turret is turrning the opposite way
    if (turrAngleBet<0){
      diffInSpd *= -1;
    }
 

    if (turretValue == 2){
      baseSPD = 100-fabs(diffInSpd);
    }
    else if (turretValue == 1){
      baseSPD = -100+fabs(diffInSpd);
    }
    else{
      baseSPD = 0;
    } 
    //---------------------this might be the reason turret is turrning the opposite way, 
    //this is not necessary because difference of angle can define the direction of rotation

    //std::cout << "\nSpeed" << diffInSpd << ", RelA:" << robotGoal.angleBetweenHorREL-(float(turretEncoder.get_position())/100/259*12) << ", bA:" << inertial.get_rotation();
    diff1 = diffInSpd + baseSPD;
    diff2 = -diffInSpd + baseSPD;
    //also put a speed controller for flywheel here, PID is not going to optimal.
    //currently, driver have to wait for flywheel to drop speed down while moving, and the amount of decceleration seems to have no difference than turnning the motor off.
    //I looked up bangbang ctl from https://wiki.purduesigbots.com/software/control-algorithms/bang-bang
    //in the description it said to have low acc, but in vex game nothing but net, sigbots used this controller for their flywheels
    //considering the simplisity and the amount of tolerance we have, this would be a good solution for now.

    flyWheel1 = angularVelocityCalc();
    flyWheel2 = angularVelocityCalc();
    delay(20);
  }
  
}