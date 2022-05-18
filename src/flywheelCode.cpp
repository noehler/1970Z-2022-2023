#include "main.h"

double degreeHope = 0;

position_t robot;
position_t homeGoal;
robotGoalRelatives robotGoal;


//returns the absolute angle between the robot and the goal relative to the field, not the robot
void angleHoriBetween(void){
  double angleB = atan((homeGoal.zpos - robot.zpos)/(homeGoal.xpos - robot.xpos));
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


void updateTurretAngle(void){
  /* getting variables */
  angleHoriBetween();
  angleVertBetween();

  /*  CONSTANTS  */
  //variable to represent the gear ratio between the motor and the rotation of the turret
  double ratioRot = 0.14285;
  double radius = 2;

  //degree that the radius goes away from the rotation point
  double degreeRad = 94;

  //difference in degree from inertial and shot out of
  double degreeOff = degreeRad - 90;

  //angle between rotation point and Target
  double degreeRotTar = robotGoal.angleBetweenHorABS;

  //angle located at point of rotation opposite to distance between shoot point and Target
  //IDFK = I dont fudging know
  double degreeIDFK = asin(sqrt(pow(robotGoal.distBetweenH, 2)-pow(radius,2))/robotGoal.distBetweenH);

  //adding angles, idk why this works, Chenghan did the math
  double goAngle = degreeRad - degreeIDFK + degreeRotTar;

  
  //converting angle to field to angle usable by robot.
  robotGoal.angleBetweenHorREL = robotGoal.angleBetweenHorABS - robot.angle;


  double angleToH = robotGoal.angleBetweenHorREL;
  double angleToV = robotGoal.angleBetweenV;


  /*  accounting for robot velocity  */
  //double purpendicularVelocity = robot.velocity / cos(robotGoal.angleBetweenHorREL + M_PI/4);

  //dividing feet by feet per second to give time taken to arrive at location
  //taking time and multiplying it my purpendicularVelocity to get how far off awway the frizbee will be
  //double feetAway = (robotGoal.distBetweenH / Ffps/cos(angleToV)) /* purpendicularVelocity*/;

  //converting from feet away to degree that needs to be added to current degree to compensate for speed of robot
  //double degreeCorrection = atan(feetAway / robotGoal.distBetweenH);
  //angleToH += degreeCorrection;


  angleToH =-angleToH*180/M_PI;
  angleToH += 180-16;

  std::cout <<"\n\n\nAngle: " << ratioRot  <<"\n" << angleToH;


  //making the turret spin to face the goal
  //turrYRot1.move_absolute(angleToH/ratioRot*4+angleShootStraight, 127);
  //turrYRot1.move_absolute(angleToH/ratioRot*4+angleShootStraight, 127);
  degreeHope = angleToH;

  while(fabs(inertial.get_heading() - angleToH) > 1 ){
    double degToGo = fabs(inertial.get_heading() - angleToH);
    int speed;
    if (degToGo > 30){
      speed = 100;
    }
    else{
      speed = 30;
    }
    if (inertial.get_heading() < angleToH){
      speed = -speed;
    }
    turrYRot1 = speed;
    turrYRot2 = speed;
    delay(30);
  }
  turrYRot1.set_brake_mode(MOTOR_BRAKE_HOLD);
  turrYRot2.set_brake_mode(MOTOR_BRAKE_HOLD);
  turrYRot1.brake();
  turrYRot2.brake();

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
  double acceleration = 7.726667;

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
  flyWheel3 = rotNeededL;
  flyWheel4 = rotNeededL;

  lcd::print(2, "DistH: %f, theta: %f, ", robotGoal.distBetweenH, robotGoal.angleBetweenV);
}
