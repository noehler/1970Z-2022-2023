#include "main.h"
#include "odometry.h"
#include "pros/optical.h"
#include "pros/rtos.h"
#include "robotConfig.h"
#define DEG2RAD M_PI/180

double goalSpeed = 0;
float wheelRad = 2.5;
float gearRatio = 9;

double angularVelocityCalc(void){
  double attackSpeed = goalSpeed/wheelRad*180/M_PI *gearRatio/600;
  //std::cout << "\nW: " << attackSpeed << ", Input: " << goalSpeed;
  return attackSpeed;
}

//tV values and what they mean
//0 is stopped
//1 is reversed
//2 is fwd
int turretValue = 0;

void turretControl(void){
  while(1){
    robotGoal.angleBetweenHorREL = (inertial.get_rotation() - robotGoal.angleBetweenHorABS);
    int baseSPD;

    static bool inertWorks = false;
    double turrAngle;
    double diffInSpd;

    if (inertWorks){
      turrAngle = inertialTurret.get_rotation();
      diffInSpd = pow(fabs(robotGoal.angleBetweenHorREL+turrAngle), 1.4/3)*18;
    }
    else{
      turrAngle = float(turretAngle.get_position())/100/259*12;
      diffInSpd = pow(fabs(robotGoal.angleBetweenHorREL-turrAngle), 1.4/3)*18;
    }
    if (radRotation == PROS_ERR_F)
    {
      turrAngle = float(turretAngle.get_position())/100/259*12;
      diffInSpd = pow(fabs(robotGoal.angleBetweenHorREL-turrAngle), 1.4/3)*18;
      alert("Turr Inert Alert");
      inertWorks = false;
    }


    if (robotGoal.angleBetweenHorREL-(float(turretAngle.get_position())/100/259*12)<0){
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
    //std::cout << "\nSpeed" << diffInSpd << ", RelA:" << robotGoal.angleBetweenHorREL-(float(turretAngle.get_position())/100/259*12) << ", bA:" << inertial.get_rotation();
    diff1 = diffInSpd+baseSPD;
    diff2 = -diffInSpd + baseSPD;
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

  shootPist = master.get_digital(DIGITAL_A);
  elevatePist = master.get_digital(DIGITAL_B);

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
