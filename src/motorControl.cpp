#include "pros/misc.h"
#include "robotConfig.h"

chassis_t chassis;

double turrControl(void){
  static double PIDSpeedSpin = 0;
  static double prevPIDSpeedSpin = 0;
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
    PIDSpeedSpin = 0.1*turrAngleBet + 0.001*prevPIDSpeedSpin*.01 + 0.1*(PIDSpeedSpin - prevPIDSpeedSpin)/.01;
    prevPIDSpeedSpin = PIDSpeedSpin;
    }else{
        diffInSpd = 0; // put that PID here
    }
    return diffInSpd;
}

double intakeControl(double diffInSpd){
    int baseSPD;
    if (chassis.intakeRunning == 2){
      baseSPD = 127-fabs(diffInSpd);
    }
    else if (chassis.intakeRunning == 1){
      baseSPD = -127+fabs(diffInSpd);
    }
    else{
      baseSPD = 0;
    } 
    return baseSPD;
}

moveToInfo_t move;
void moveTo(){
  if (move.reset) {
    move.dist = 0;          // change of position
    move.distR = 0;         // chagne of right postion
    move.distL = 0;         // change of left position
    move.PIDSS = 0;         // PID turning speed
    move.PIDFW = 0;         // PID moveforward speed
    move.PIDSpeedL = 0;     // PID leftside speed
    move.PIDSpeedR = 0;     // PID rightside speed
    move.prevPIDFW = 0;     // PID moveforward speed at t = -1
    move.prevPIDSS = 0;     // PID turning speed at t = -1
    move.PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
    move.PIDSSFLAT = 0;     // variable used for keeping turning speed < 20
    move.targetHeading = 0; // variable for for calculating first turning.
    move.reset = false;
  }
  double currentheading = -inertial.get_rotation();
  /*
  function logic:
  find errors of position, turn to target if robot cannot move in a arc to
  it, than move to target in a arc. tracking center of robot is at the
  center of two tracking wheels do not recomand using this funciton with
  SpinTo() function. perferd to have a sperate thread for calculating live
  position, than just take out codes from line 41 to line 48
  */
  double etx = move.moveToxpos - robot.xpos;//change of x
  double ety = move.moveToypos - robot.ypos;//change of y
  double dist = sqrt(pow(etx, 2) + pow(ety, 2));
  double et = dist * 41.6696578277;
  double r = ety / dist;
  if ((etx) < 0) {
  move.targetHeading = 180 - asin(r) * 180 / M_PI;
  } else if (et == 0) {
  move.targetHeading = currentheading * 180 / M_PI;
  } else {
  move.targetHeading = asin(r) * 180 / M_PI;
  }
  if (move.moveToforwardToggle == -1) {
  move.targetHeading += 180;
  }
  while (move.targetHeading < 0) {
  move.targetHeading += 360;
  }
  while (move.targetHeading > 360) {
  move.targetHeading -= 360;
  }
  while (currentheading * 180 / M_PI < 0) {
  move.targetHeading = (currentheading * 180 / M_PI + 360) * 0.01745329251;
  }
  while (currentheading * 180 / M_PI > 360) {
  move.targetHeading = (currentheading * 180 / M_PI - 360) * 0.01745329251;
  }
  move.ets = move.targetHeading - currentheading * 180 / M_PI;
  while (move.ets < -180) {
  move.ets += 360;
  }
  while (move.ets > 180) {
  move.ets -= 360;
  }
  move.PIDSS = 1 * move.ets + 0.1 * move.prevPIDSS * .01 + 1 * (move.PIDSS - move.prevPIDSS) / .01;
  if (fabs(move.ets) < move.errtheta) {
    move.PIDFW = move.moveToforwardToggle * (3 * et + 1 * move.prevPIDFW * .01 + 0.4 * (move.PIDFW - move.prevPIDFW) / .01);
  } else {
    move.PIDFW = 0;
  }

  move.PIDSSFLAT = move.PIDSS;
  move.PIDFWFLAT = move.PIDFW;
  if (move.PIDFWFLAT >= move.speed_limit) {
    move.PIDFWFLAT = move.speed_limit;
  }
  if (move.PIDFWFLAT <= -move.speed_limit) {
    move.PIDFWFLAT = -move.speed_limit;
  }
  if (move.PIDSSFLAT >= 2 * move.speed_limit) {
    move.PIDSSFLAT = 2 * move.speed_limit;
  }
  if (move.PIDSSFLAT <= -2 * move.speed_limit) {
    move.PIDSSFLAT = -2 * move.speed_limit;
  }
  if (move.moveToforwardToggle) {
    move.PIDSpeedR = move.PIDFWFLAT + move.PIDSSFLAT;
    move.PIDSpeedL = move.PIDFWFLAT - move.PIDSSFLAT;

  } else {
    move.PIDSpeedR = move.PIDFWFLAT + move.PIDSSFLAT;
    move.PIDSpeedL = move.PIDFWFLAT - move.PIDSSFLAT;
  }
  if (dist < 3) {
    move.reset = true;
    if (move.Stop_type == 1) {
        //motor stop (hold)
        chassis.driveTrain.leftSpd = 0;
        chassis.driveTrain.rightSpd = 0;
        chassis.driveTrain.mechSpd = 0;
        lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    } else {
        //motor stop (coast)
        chassis.driveTrain.leftSpd = 0;
        chassis.driveTrain.rightSpd = 0;
        chassis.driveTrain.mechSpd = 0;
        lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);

    }
    //continue;
  }
  //output motor speeds
  chassis.driveTrain.leftSpd = move.PIDFW + move.PIDSS;
  chassis.driveTrain.rightSpd = move.PIDFW - move.PIDSS;
  chassis.driveTrain.mechSpd = 0;

  move.prevPIDSS = move.PIDSS;
  move.prevPIDFW = move.PIDFW;
}

void motorControl(void){
  while(runLoop){
    //getting speeds that diff needs to run at
    double diffInSpd = turrControl();
    int baseSPD = intakeControl(diffInSpd);

    //also put a speed controller for flywheel here, PID is not going to optimal.
    //currently, driver have to wait for flywheel to drop speed down while moving, and the amount of decceleration seems to have no difference than turnning the motor off.
    //I looked up bangbang ctl from https://wiki.purduesigbots.com/software/control-algorithms/bang-bang
    //in the description it said to have low acc, but in vex game nothing but net, sigbots used this controller for their flywheels
    //considering the simplisity and the amount of tolerance we have, this would be a good solution for now.

    diff1 = diffInSpd + baseSPD;
    diff2 = -diffInSpd + baseSPD;

    double flyWheelVelocity = flyWheel1.get_actual_velocity()/2 + flyWheel2.get_actual_velocity()/2;
    if (angularVelocityCalc()-flyWheelVelocity<-5){
      flyWheel1 = 127;
      flyWheel2 = 127;
    } else if (angularVelocityCalc()-flyWheelVelocity>5){
      flyWheel1 = -127;
      flyWheel2 = -127;
    } else{
      flyWheel1 = angularVelocityCalc();
      flyWheel2 = angularVelocityCalc();
    }

    if (chassis.driveTrain.leftSpd != 0 || chassis.driveTrain.rightSpd != 0 ||chassis.driveTrain.mechSpd != 0){
      lfD.move(chassis.driveTrain.leftSpd + chassis.driveTrain.mechSpd); 
      lbD.move(chassis.driveTrain.leftSpd - chassis.driveTrain.mechSpd);
      rfD.move(chassis.driveTrain.rightSpd - chassis.driveTrain.mechSpd);
      rbD.move(chassis.driveTrain.rightSpd + chassis.driveTrain.mechSpd);
    }
    else{
      lfD.brake();
      lbD.brake();
      rfD.brake();
      rbD.brake();
    }

    delay(20);
    if (competition::is_autonomous()){
      moveTo();
    }
  }
  
}
