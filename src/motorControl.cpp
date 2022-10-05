#include "pros/misc.h"
#include "robotConfig.h"

double turrControl(void){
    double turrAngle = -float(turretEncoder.get_position())/100/259*12;
    double turrAngleABS  = inertial.get_rotation() + turrAngle;

    //std::cout << "\nTAngleRel: " << turrAngle;  
    bool dtb = false;

    static double diffInSpd;

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
    if (turrAngleBet<0){
      diffInSpd *= -1;
    }
    return diffInSpd;
}

double intakeControl(double diffInSpd){
    int baseSPD;
    if (turretValue == 2){
      baseSPD = 100-fabs(diffInSpd);
    }
    else if (turretValue == 1){
      baseSPD = -100+fabs(diffInSpd);
    }
    else{
      baseSPD = 0;
    } 
    return baseSPD;
}

void motorControl(void){
  while(runLoop){
    //getting speeds that diff needs to run at
    double diffInSpd = turrControl();
    int baseSPD = intakeControl(diffInSpd);

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
    if (competition_is_autonomous()){
      moveTo();
    }
  }
  
}
void moveTo(){
      if (resetMoveTo) {
        dist = 0;          // change of position
        distR = 0;         // chagne of right postion
        distL = 0;         // change of left position
        PIDSS = 0;         // PID turning speed
        PIDFW = 0;         // PID moveforward speed
        PIDSpeedL = 0;     // PID leftside speed
        PIDSpeedR = 0;     // PID rightside speed
        prevPIDFW = 0;     // PID moveforward speed at t = -1
        prevPIDSS = 0;     // PID turning speed at t = -1
        PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
        PIDSSFLAT = 0;     // variable used for keeping turning speed < 20
        targetHeading = 0; // variable for for calculating first turning.
        resetMoveTo = false;
      }
      /*
      function logic:
        find errors of position, turn to target if robot cannot move in a arc to
      it, than move to target in a arc. tracking center of robot is at the
      center of two tracking wheels do not recomand using this funciton with
      SpinTo() function. perferd to have a sperate thread for calculating live
      position, than just take out codes from line 41 to line 48
      */
      double etx = moveToxpos - posx;//change of x
      double ety = moveToypos - posy;//change of y
      double dist = sqrt(pow(etx, 2) + pow(ety, 2));
      double et = dist * 41.6696578277;
      double r = ety / dist;
      if ((etx) < 0) {
        targetHeading = 180 - asin(r) * 180 / M_PI;
      } else if (et == 0) {
        targetHeading = currentheading * 180 / M_PI;
      } else {
        targetHeading = asin(r) * 180 / M_PI;
      }
      if (moveToforwardToggle == -1) {
        targetHeading += 180;
      }
      while (targetHeading < 0) {
        targetHeading += 360;
      }
      while (targetHeading > 360) {
        targetHeading -= 360;
      }
      while (currentheading * 180 / M_PI < 0) {
        targetHeading = (currentheading * 180 / M_PI + 360) * 0.01745329251;
      }
      while (currentheading * 180 / M_PI > 360) {
        targetHeading = (currentheading * 180 / M_PI - 360) * 0.01745329251;
      }
      ets = targetHeading - currentheading * 180 / M_PI;
      while (ets < -180) {
        ets += 360;
      }
      while (ets > 180) {
        ets -= 360;
      }
      PIDSS = 1 * ets + 0.1 * prevPIDSS * .01 + 1 * (PIDSS - prevPIDSS) / .01;
      if (fabs(ets) < errtheta) {
          PIDFW = moveToforwardToggle * (3 * et + 1 * prevPIDFW * .01 + 0.4 * (PIDFW - prevPIDFW) / .01);
      } else {
        PIDFW = 0;
      }

      PIDSSFLAT = PIDSS;
      PIDFWFLAT = PIDFW;
      if (PIDFWFLAT >= speed_limit) {
        PIDFWFLAT = speed_limit;
      }
      if (PIDFWFLAT <= -speed_limit) {
        PIDFWFLAT = -speed_limit;
      }
      if (PIDSSFLAT >= 2 * speed_limit) {
        PIDSSFLAT = 2 * speed_limit;
      }
      if (PIDSSFLAT <= -2 * speed_limit) {
        PIDSSFLAT = -2 * speed_limit;
      }
      if (moveToforwardToggle) {
        PIDSpeedR = PIDFWFLAT + PIDSSFLAT;
        PIDSpeedL = PIDFWFLAT - PIDSSFLAT;

      } else {
        PIDSpeedR = PIDFWFLAT + PIDSSFLAT;
        PIDSpeedL = PIDFWFLAT - PIDSSFLAT;
      }
      if (dist < 3) {
        resetMoveTo = true;
        if (Stop_type == 1) {
          //motor stop (hold)
        } else {
          //motor stop (coast)
        }
        continue;
      }
      //output motor speeds

      prevPIDSS = PIDSS;
      prevPIDFW = PIDFW;
      task::sleep(10);
}