#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"
#include "main.h"
chassis_t chassis;

bool underRoller = 0;
double angdiff;
double turrControl(void){
  static double PIDPosition = 0;
  static double PIDVelocity = 0;
  static double T = 0;
  static double previousT=0;  
  static double PIDscalar = 1.5;
  static double gyroScalar = 21.5833333;
  static double chassisScalar = 21.5833333;
  static double turPredicScalar = 21.5833333;
  T = float(millis())/1000 - previousT;
  previousT+=T;
  /*if (!competition::is_disabled() && !competition::is_autonomous()){
    robotGoal.angleBetweenHorABS = robot.angle + 180;
  }*/
  angdiff = robotGoal.angleBetweenHorABS - robot.turAng;
  if (angdiff > 180){
      angdiff -= 360;
  }
  else if( angdiff < -180){
      angdiff += 360;
  }
  if (fabs(angdiff) < 2){
    angdiff = 0;
  }
  //feed forward code todo here,
  //chassie rotation rate = chassie change of angle in last cycle / elapsed time of last cycle (odom loop)
  //turret target speed = turret ang dif / elapsed time of last cycle (turret twister loop)
  //turret target speed = -chassie rotation rate (because turret need to go to opposite of chassie rotation) + turret target speed
  //feed above calculated speed into a PID

  //slowing down turret when nothing is loaded or on deck so intake can run faster
  //if (deckLoaded.get_value() > 1900){


  static double IPIDvel = 0;
  static double previousveldiff = 0;
  static double IPIDang = 0;
  static double previousangdiff = 0;
  //double PIDinPut = 10*(gyroScalar*T/2*(inertial.get_gyro_rate().z)-robot.wVelocity*chassisScalar + turPredicScalar*robot.turvelocity)*T + 0*angdiff;
  if (!competition::is_disabled()){
    IPIDang += angdiff;
    PIDPosition =(PID.turret.p*angdiff + PID.turret.i*IPIDang + PID.turret.d*(angdiff - previousangdiff));
    previousangdiff = angdiff;
    double veldiff = gyroScalar*T*(inertial.get_gyro_rate().z)-robot.wVelocity*chassisScalar + turPredicScalar*robot.turvelocity+PIDPosition*PIDscalar + 0.025*recoilPrevent*goalSpeed;
    IPIDvel += veldiff;
    PIDVelocity =(0.415*veldiff + 0.135*IPIDvel*.01 + 2.6*(veldiff - previousveldiff));
    previousveldiff = veldiff;
    if (fabs(angdiff) == 0 || PIDPosition==0){
      IPIDang = 0;
    }
    if (fabs(veldiff)<0.1){
      IPIDvel = 0;
    }

    /*if (deckLoaded.get_value() > 1000){
      PIDSpeedSpin = 0;
    }*/

    //}else{
    //    diffInSpd = 0; // put that PID here
    //}
    /*logVals("VEL" , PIDVelocity);
    logVals("time", millis());
    logVals("VD", veldiff);
    logVals("PP", PIDPosition);
    logVals("AD", double(angdiff));
    logVals();*/
  }
  else{
    PIDVelocity = 0;
  }
  
  
  if (opticalSensor.get_proximity() > 200){
    PIDVelocity = 0;
    underRoller = true;
  }
  else{
    underRoller = false;
  }
  if (chassis.intakeRunning != 0){
    PIDVelocity = 0;
    IPIDvel = 0;
    IPIDang = 0;
  }

  return PIDVelocity;
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

  if (underRoller){
    baseSPD *= .5;
    baseSPD -= (chassis.driveTrain.leftSpd + chassis.driveTrain.rightSpd)/2;
  }
  return baseSPD;
}

class moveToInfoInternal_t{
  public:
    double moveToxpos, moveToypos, targetHeading, ets, speed_limit=100, errtheta=5;
    double dist = 0;          // change of position
    double distR = 0;         // chagne of right postion
    double distL = 0;         // change of left position
    double PIDSS = 0;         // PID turning speed
    double PIDFW = 0;         // PID moveforward speed
    double PIDSpeedL = 0;     // PID leftside speed
    double PIDSpeedR = 0;     // PID rightside speed
    double prevPIDFW = 0;     // PID moveforward speed at t = -1
    double prevPIDSS = 0;     // PID turning speed at t = -1
    double PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
    double PIDSSFLAT = 0;   
    int moveToforwardToggle = 1, Stop_type = 2;
    double tolerance=5;
};
moveToInfoExternal_t move;
void moveTo(void){
  static moveToInfoInternal_t moveI;
  
  static double IPIDSS = 0;
  static double previousets = 0;
  static double IPIDfw = 0;
  static double previouset = 0;
  if (move.resetMoveTo) {
    IPIDSS = 0;
    previousets = 0;
    IPIDfw = 0;
    previouset = 0;
    moveI.dist = 0;          // change of position
    moveI.distR = 0;         // chagne of right postion
    moveI.distL = 0;         // change of left position
    moveI.PIDSS = 0;         // PID turning speed
    moveI.PIDFW = 0;         // PID moveforward speed
    moveI.PIDSpeedL = 0;     // PID leftside speed
    moveI.PIDSpeedR = 0;     // PID rightside speed
    moveI.prevPIDFW = 0;     // PID moveforward speed at t = -1
    moveI.prevPIDSS = 0;     // PID turning speed at t = -1
    moveI.PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
    moveI.PIDSSFLAT = 0;     // variable used for keeping turning speed < 20
    // variable for for calculating first turning.
    move.resetMoveTo = false;
  }
  double currentheading =robot.angle/180*M_PI;
  if (currentheading == PROS_ERR_F)
  {
    // JLO - handle error and exit, we can't continue
    std::cout << "\n headingFudge";
    return;
  }
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
  double et = dist * 10;

  //std::cout << "\n Hi5";
  move.targetHeading = atan(ety/etx);
  if (etx<0){
    move.targetHeading +=M_PI;
  } else if(etx ==0){
    move.targetHeading = M_PI/2*(fabs(ety)/ety);
  }
  if (move.moveToforwardToggle == -1){
    move.targetHeading +=M_PI;
  }
  moveI.ets = move.targetHeading - currentheading;
  if (moveI.ets < -M_PI) {
  moveI.ets += 2*M_PI;
  }
  if (moveI.ets > M_PI) {
  moveI.ets -= 2*M_PI;
  }

  //std::cout <<"\nxpos"<<robot.xpos<<" y:"<<robot.ypos<<" ang:"<<robot.angle;
  //std::cout <<"\na:"<<move.ets<<" tarx:"<<move.moveToxpos<<" tary:"<<move.moveToypos;
  
  moveI.ets = moveI.ets*180/M_PI;
  IPIDSS += moveI.ets;
  IPIDfw += et;
  if (move.moveToforwardToggle == 1){
    moveI.PIDSS = 3 * moveI.ets + 0.1 * IPIDSS * .01 + 0.3 * (moveI.ets - previousets);
  }
  else{
    moveI.PIDSS = 3 * moveI.ets + 0.1 * IPIDSS * .01 + 0.3 * (moveI.ets - previousets);
  }
  if (fabs(moveI.ets) < 10) {
    if (move.moveToforwardToggle == 1){
      moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
    }
    else{
      moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
    }
  } else {
    moveI.PIDFW = 0;
  }
  previousets = moveI.ets;
  previouset = et;
  moveI.PIDSSFLAT = moveI.PIDSS;
  moveI.PIDFWFLAT = moveI.PIDFW;
  if (moveI.PIDFWFLAT >= move.speed_limit) {
    moveI.PIDFWFLAT = move.speed_limit;
  }
  if (moveI.PIDFWFLAT <= -move.speed_limit) {
    moveI.PIDFWFLAT = -move.speed_limit;
  }
  if (moveI.PIDSSFLAT >= 2 * move.speed_limit) {
    moveI.PIDSSFLAT = 2 * move.speed_limit;
  }
  if (moveI.PIDSSFLAT <= -2 * move.speed_limit) {
    moveI.PIDSSFLAT = -2 * move.speed_limit;
  }
  if (move.moveToforwardToggle) {
    moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
    moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;

  } else {
    moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
    moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
  }
  if (dist < move.tolerance) {
    move.resetMoveTo = true;
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
  } else {
  chassis.driveTrain.leftSpd = moveI.PIDSpeedL;
  chassis.driveTrain.rightSpd = moveI.PIDSpeedR;
  chassis.driveTrain.mechSpd = 0;
  }
  //output motor speeds

  //std::cout << "\net: " << dist << ", ets: " << move.ets;
  //std::cout << "\npidfw: " << move.PIDFW << ", pidss: " << move.PIDSS;

  

  //std::cout << "\n(" << robot.xpos << "," << robot.ypos << ")";
}

void spinRoller(void){
  if (chassis.isSpinner == true){
    double redVal = opticalSensor.get_rgb().red;
    double blueVal = opticalSensor.get_rgb().blue;
    //0 is blue
    //1 is red
    bool colorDown;
    if (redVal < blueVal){
      colorDown = 0;
    }
    else{
      colorDown = 1;
    }
    if (chassis.teamColor == colorDown){
      chassis.intakeRunning = 2;
    }
    else{
      chassis.intakeRunning = 0;
    }

  }
}

double diffFlyWheelW;
double flyWheelW;
double flyPIDP(void){
  static double IPIDang = 0;
  if (competition::is_disabled()){
    IPIDang = 0;
  }
  double flyWVolt;
  flyWheelW =(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity())/2;
  diffFlyWheelW = angularVelocityCalc()-flyWheelW;
  static double prevFWdiffSPD = angularVelocityCalc();

  IPIDang += diffFlyWheelW;
  double prop = PID.flyWheel.p*diffFlyWheelW;
  double integ = IPIDang*PID.flyWheel.i;
  double deriv = PID.flyWheel.d*(diffFlyWheelW - prevFWdiffSPD);
  double prop2 = PID.flyWheel.p2 * flyWheel1.get_actual_velocity();
  prevFWdiffSPD = diffFlyWheelW;
  flyWVolt = 12000.0/127*(prop + integ + deriv + prop2);
  if (flyWVolt > 12000){
    flyWVolt = 12000;
  }
  if (flyWVolt < -12000){
    flyWVolt = -12000;
  } 
  if (fabs(diffFlyWheelW)<1&&flyWVolt==0){
    IPIDang = 0;
  }

  return flyWVolt;
}

bool readyFire = false;;
void waitShootuc(void){
  while(1){
    while (readyFire != true){
      delay(20);
    }
    /*while (fabs(angularVelocityCalc() - flyWheelW) > 5 || fabs(angdiff) > 3){
      delay(20);
    }*/
    shootPiston.set_value(true);
    recoilPrevent = 1;
    delay(250);
    recoilPrevent = 0;
    delay(250);
    shootPiston.set_value(false);
    readyFire = false;
    delay(20);
  }
}


void waitShoot(void){
  while (fabs(angularVelocityCalc() - flyWheelW) > 5 || fabs(angdiff) > 3){
    delay(20);
  }
  shootPiston.set_value(true);
  recoilPrevent = 1;
  delay(250);
  recoilPrevent = 0;
  delay(250);
  shootPiston.set_value(false);
}

void motorControl(void){
  while(1){
    //std::cout << "\n(t*cos(" << -inertial.get_heading()/180*M_PI <<")+" << robot.xpos <<",t*sin(" << -inertial.get_heading()/180*M_PI << ")+" << -inertial.get_heading()/180*M_PI << ")";
    //getting speeds that diff needs to run at
    double diffInSpd = 0;//turrControl();
    int baseSPD = intakeControl(diffInSpd);
    double FlyWVolt, prevFlyWVolt;

    diff1 = diffInSpd + baseSPD;
    diff2 = -diffInSpd + baseSPD;
    
    double flyWVolt = 0;//flyPIDP();

    flyWheel1.move_voltage(flyWVolt); 
    flyWheel2.move_voltage(flyWVolt); 

    if ((chassis.driveTrain.leftSpd != 0 || chassis.driveTrain.rightSpd != 0 ||chassis.driveTrain.mechSpd != 0) && chassis.driveTrain.running){
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
    //std::cout << "\n motorControl entered";

    spinRoller();

    if (competition::is_autonomous()){
      moveTo();
      if (usd::is_installed()){
        //std::cout << "(" << robot.xpos << "," << robot.ypos << ")\n";
        outPosSDCARD();
        outValsSDCard();
      }
    }
    static int prevTime = millis();
		loopTimes[1] = millis() - prevTime;
		prevTime = millis();
    delay(20);
  }

}
