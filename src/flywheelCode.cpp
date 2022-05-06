#include "main.h"

position_t robot;
position_t homeGoal;

class robotGoalRelatives {
  public:
    //storing the absolute and relative horizontal angle between goal and robot
    double angleBetweenHorABS;
    double angleBetweenHorREL;

    //storing angle between turret and goal
    double angleBetweenV;

    //storing the distance the frizbee will travel horizontally
    double distBetweenH;
};
robotGoalRelatives robotGoal;


//returns the absolute angle between the robot and the goal relative to the field, not the robot
void angleHoriBetween(void){
  double angleB = atan((homeGoal.zpos - robot.zpos)/(homeGoal.xpos - robot.xpos));
  robotGoal.angleBetweenHorABS = angleB;
}


//finds angle that the goal is above to robot and dist between
void angleVertBetween(void){
  double turretHeight = 10;

  double distH = sqrt(
    pow(robot.xpos - homeGoal.xpos, 2) +
    pow(robot.zpos - homeGoal.zpos, 2)
  );
  distH = robotGoal.distBetweenH;

  double distVert = homeGoal.ypos - (robot.ypos + turretHeight);

  double angleV = atan(distVert / distH);
  robotGoal.angleBetweenV = angleV;
}


void updateTurretAngles(void){
  /*  CONSTANTS  */
  //variable to represent the gear ratio between the motor and the rotation of the turret
  double ratio = 1/3;

  //variable to represent the velocity frizbees are shot at in ft/s
  double Ffps = 10;

  /* getting variables */
  angleHoriBetween();

  //converting angle to field to angle usable by robot.
  robotGoal.angleBetweenHorREL = robotGoal.angleBetweenHorABS - robot.angle;

  double angleToH = robotGoal.angleBetweenHorREL;
  double angleToV = robotGoal.angleBetweenV;

  /*  accounting for robot velocity  */
  double purpendicularVelocity = robot.velocity / cos(robotGoal.angleBetweenHorREL + M_PI/4);

  //dividing feet by feet per second to give time taken to arrive at location
  //taking time and multiplying it my purpendicularVelocity to get how far off awway the frizbee will be
  double feetAway = (robotGoal.distBetweenH / Ffps/cos(angleToV)) * purpendicularVelocity;

  //converting from feet away to degree that needs to be added to current degree to compensate for speed of robot
  double degreeCorrection = atan(feetAway / robotGoal.distBetweenH);
  angleToH += degreeCorrection;

  /* getting angle on x axis */




  //making the turret spin to face the goal
  turrYRot.move_absolute(angleToH*ratio, 127);

}
