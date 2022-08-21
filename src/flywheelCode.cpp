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
  robot.velocity = velocityCalc();
  double angleB = atan((homeGoal.zpos - robot.zpos + (robot.velocity*sin(robot.angle)*timeInAir()))/(homeGoal.xpos - robot.xpos + (robot.velocity*cos(robot.angle)*timeInAir())));
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