#include "devFunctions.h"
#include "main.h"
#include "odometry.h"
#include "pros/optical.h"
#include "pros/rtos.h"
#include "robotConfig.h"
#define DEG2RAD M_PI/180

double goalSpeed = 0;
float wheelRad = 3;
float gearRatio = 9;

double flySpdMult = 1;
double angularVelocityCalc(void){
  double attackSpeed;
  if (goalSpeed < 228){
  attackSpeed = ((goalSpeed*0.3240) + 11)*flySpdMult;
  } else {
  attackSpeed = ((goalSpeed*1.142) - 225.4)*flySpdMult;
  }
  //std::cout << "\nW: " << attackSpeed << ", Input: " << goalSpeed;
  return attackSpeed;
}

//tV values and what they mean
//0 is stopped
//1 is reversed
//2 is fwd
int turretValue = 0;

void turretControl(void){
  while(runLoop){
    double turrAngle = -float(turretEncoder.get_position())/100/259*12;
    double turrAngleABS  = inertial.get_rotation() + turrAngle;

    static double diffInSpd;
    //std::cout << "\nTAngleRel: " << turrAngle;  
    bool dtb = false;

    /*if (fabs(turrAngleABS + inertialTurret.get_rotation()) > 3 && abs(turretEncoder.get_velocity()) < 300){
      turretEncoder.set_position((- inertial.get_rotation() + inertialTurret.get_rotation() )*100*259/12);
      dtb = 1;
    }
    std::cout << "\ndiff: " << fabs(turrAngleABS + inertialTurret.get_rotation()) << ",     IturrAngle: " << inertialTurret.get_rotation() << ",    AbsRot: " << turrAngleABS
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
    diffInSpd = pow(fabs(turrAngleBet), 1.4/3)*18; // put that PID here

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

//defining constants
float g = 386.08858267717;
double a = -pow(g,2)*.25;

//means iterative time based turret rotation calculator
void singSameOldSongTimeTurretTwister(void){
  //define quartic equation terms  
  double c = pow(robot.xVelocity, 2) + pow(robot.yVelocity, 2) - robotGoal.dz * g;
  double d = -2 * robotGoal.dx * robot.xVelocity - 2 * robotGoal.dy * robot.yVelocity;
  double e = pow(robotGoal.dx, 2) + pow(robotGoal.dy, 2) - pow(robotGoal.dz, 2);
  double D = 1000000000000;
  double T = 0.1;

  bool close_enough = false;
  while (close_enough != true){
    if (D > 10000){
      T += 0.1;
    }  
    else if (D > 1){
      T += 0.001;
    }
    else{
      close_enough = true;
    }
    D = a * pow(T, 4) + c * pow(T, 2) + d * T + e;
  }
  
  double P1 = robotGoal.dy - robot.yVelocity * T;
  double P2 = robotGoal.dx - robot.xVelocity * T;
  double Tar_ang = atan(P1 / P2);
  if (P2 < 0){
    Tar_ang = Tar_ang + M_PI;
  }
  /*while(Tar_ang < 0){
    Tar_ang+=2*M_PI;
  }
  while(Tar_ang > 2*M_PI){
    Tar_ang-=2*M_PI;
  }*/

  // why take this mod calculation out?

  //std::cout<< "\nAngle: " << Tar_ang;
  double P3 = cos(Tar_ang) * 0.707106781187 * T;
  double V_disk = P2 / P3;

  //outputting calculated values
  robotGoal.angleBetweenHorABS = Tar_ang *180/M_PI;
  goalSpeed = V_disk;
  //std::cout << "\nAngle:" << robotGoal.angleBetweenHorABS;
}

void liftConrol(void){
  static int elevateTime = 0;
  //static int shootTime = 0;
  static bool elevatePist = true;
  static bool shootPist = true;

  shootPist = master.get_digital(DIGITAL_L2);
  elevatePist = master.get_digital(DIGITAL_L1);

  /*if (deckLoaded.get_value() <1900 && upLoaded.get_value() > 1900 && shootPist == false && millis() - elevateTime > 400 ){
    elevateTime = millis();
    elevatePist = true;
  }
  //std::cout << "\n UP: " << upLoaded.get_value() << ", Deck: " << deckLoaded.get_value();

  if (millis() - elevateTime > 300){
    elevatePist = false;
  }*/
  
  shootPiston.set_value(shootPist);
  elevatePiston.set_value(elevatePist);

  if (master.get_digital(DIGITAL_R1)){
    turretValue = 2;
  }
  else if (master.get_digital(DIGITAL_R2)){
    turretValue = 1;
  }
  else{
    turretValue = 0;
  }
  
}
