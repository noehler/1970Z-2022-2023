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
  if (goalSpeed < 279.72){
  attackSpeed = ((goalSpeed*0.3184) + 11.5)*flySpdMult;
  } else {
  attackSpeed = ((goalSpeed*0.7544) - 110.5)*flySpdMult;
  }
  //std::cout << "\nW: " << attackSpeed << ", Input: " << goalSpeed;
  return attackSpeed;
}

//defining constants
float g = 386.08858267717;
double a = -pow(g,2)*.25;

bool overrideSSOSTTT = false;
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
  double Tar_ang = 0;
  if (P2 == 0){
    if (P1 > 0){
      Tar_ang = M_PI/2;
    } else {
      Tar_ang = -M_PI/2;
    }
  } else {
    Tar_ang = atan(P1 / P2);
    if (P2 < 0){
      Tar_ang = Tar_ang + M_PI;
    } else {
      Tar_ang = Tar_ang +2*M_PI;
    }
  }

  //std::cout<< "\nAngle: " << Tar_ang;
  double P3 = cos(Tar_ang) * 0.707106781187 * T;
  double V_disk = P2 / P3;
  double turOfCenterOffset = 0; // offcenter offset, not tested yet
  //outputting calculated values
  if (!overrideSSOSTTT){
    robotGoal.angleBetweenHorABS = Tar_ang *180/M_PI + targetAngleOffest+turOfCenterOffset;
  }
  else{
    robotGoal.angleBetweenHorABS = robot.chaIntAng;
  }
  goalSpeed = V_disk;
  robot.turvelocity = (robot.xVelocity*P1-robot.yVelocity*P2)/(pow(P1,2)+pow(P2,2));
}

void liftConrol(void){
  static int elevateTime = 0;
  //static int shootTime = 0;
  static bool elevatePist = true;
  static bool shootPist = true;

  shootPist = master.get_digital(DIGITAL_L1)+ sidecar.get_digital(DIGITAL_L2);
  recoilPrevent = master.get_digital(DIGITAL_L1)+ sidecar.get_digital(DIGITAL_L2);
  elevatePist = master.get_digital(DIGITAL_L2)+ sidecar.get_digital(DIGITAL_L1);
  
  shootPiston.set_value(shootPist);
  elevatePiston.set_value(elevatePist);

  if (master.get_digital(DIGITAL_R1)){
    chassis.intakeRunning = 2;
  }
  else if (master.get_digital(DIGITAL_R2)){
    chassis.intakeRunning = 1;
  }
  else{
    chassis.intakeRunning = 0;
  }
}
