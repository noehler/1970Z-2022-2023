#include "main.h"
#include "odometry.h"
#include "robotConfig.h"

double degreeHope = 0;

double timeInAir(void){
  double speed = 10;
  double time = robotGoal.distBetweenH / speed;
  return time;
}

//returns the absolute angle between the robot and the goal relative to the field, not the robot, accounting for velocity
void angleHoriBetween(void){
  //grobot.velocity = velocityCalc();
  double angleB = atan((homeGoal.zpos - robot.zpos + (robot.velocity*sin(robot.angle)*timeInAir()))/(homeGoal.xpos - robot.xpos + (robot.velocity*cos(robot.angle)*timeInAir())));
  angleB = atan((homeGoal.zpos - robot.zpos)/(homeGoal.xpos - robot.xpos))*180/M_PI;
  robotGoal.angleBetweenHorABS = angleB;
}


//finds angle that the goal is above to robot and dist between
void angleVertBetween(void){
  double turretHeight = 10;

  double distH = sqrt(fabs(
    pow(robot.xpos - homeGoal.xpos, 2) +
    pow(robot.zpos - homeGoal.zpos, 2)
  ));

  robotGoal.distBetweenH = distH;
  turretHeight = 0;


  double distVert = homeGoal.ypos - (robot.ypos + turretHeight);
  //std::cout << "\nVertB: " << distVert;

  double angleV = atan(distVert / distH);
  robotGoal.angleBetweenV = angleV;
  //std::cout << "sqrt((" + std::to_string(robot.xpos) + "-" + std::to_string(homeGoal.xpos) + ")^2" + "+" + "(" + std::to_string(robot.zpos) + "-" + std::to_string(homeGoal.zpos) + ")^2) = " + std::to_string(distH) + "\n";
}

double angularVelocityCalc(double angle, double distMult, double heightMult, double velPow){

  double attackSpeed = robotGoal.distBetweenH * distMult * (3/fabs(robotGoal.distBetweenH*tan(robotGoal.angleBetweenV))) * heightMult * pow(10/robotGoal.distBetweenH, velPow);
  std::cout << "\nDegree: " << robotGoal.angleBetweenV;
  return attackSpeed;

}

void turretSpeed(void){
  /* constants */
  double highAngle = M_PI/3;
  double lowAngle = M_PI/4;

  //setting constants for teting;
  lowAngle = 0;

  angleVertBetween();

  /* calculating speed and attack angle */
  double attackHigh = angularVelocityCalc(highAngle, 57, 1, .5);
  double attackLow = angularVelocityCalc(lowAngle, 57, 1, .5);


  attackLow = robotGoal.distBetweenH * 57 * (3/fabs(robotGoal.distBetweenH*tan(robotGoal.angleBetweenV))) * (sqrt((10/robotGoal.distBetweenH)));

  double rad = 2.5/12;
  double rotNeededH = attackHigh/rad/49;
  double rotNeededL = attackLow/rad/49;
  std::cout << "\n\n" + std::to_string(rotNeededL) + "\n\n";

  flyWheel1 = rotNeededL;
  flyWheel2 = rotNeededL;

  lcd::print(2, "DistH: %f, theta: %f, ", robotGoal.distBetweenH, robotGoal.angleBetweenV);
}

void turretAngleTo(void){
  //calculating the angle to drive the turret to
  angleHoriBetween();
  robotGoal.angleBetweenHorREL = robotGoal.angleBetweenHorABS - inertial.get_heading();

}

//defining constants
float g = 386.08858267717;
double a = -pow(g,2)*.25;

void singSameOldSongTimeTurretTwister(void){
  //define quartic equation terms
  double c = pow(robot.xVelocity, 2) + pow(robot.yVelocity, 2) - robotGoal.dx * g;
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
  //return V_disk, Tar_ang*180/np.pi, T;
  //idk what those things are
}
