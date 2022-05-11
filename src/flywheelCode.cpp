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


void updateTurretAngle(void){
  /*  CONSTANTS  */
  //variable to represent the gear ratio between the motor and the rotation of the turret
  double ratio = 1/3;
  double Ffps = 10;

  /* getting variables */
  angleHoriBetween();
  angleVertBetween();


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


  //making the turret spin to face the goal
  turrYRot.move_absolute(angleToH*ratio, 127);

}

void turretSpeed(void){
  /* constants */
  double highAngle = M_PI/3;
  double lowAngle = M_PI/4;
  double acceleration = 7.726667;

  //setting constants for teting;
  robot.xpos = 0;
  robot.ypos = 3;
  robot.zpos = 0;
  homeGoal.xpos = 10;
  homeGoal.ypos = 0;
  homeGoal.zpos = 0;
  lowAngle = 0;

  angleVertBetween();

  /* calculating speed and attack angle */
  double attackHigh = sqrt((acceleration * robotGoal.distBetweenH) /
                          fabs(2*tan(robotGoal.angleBetweenV - highAngle)))        * cos(robotGoal.angleBetweenV) / cos(highAngle);
  double attackLow = sqrt((acceleration * robotGoal.distBetweenH) /
                          fabs(2*tan(robotGoal.angleBetweenV - lowAngle)))         * cos(robotGoal.angleBetweenV) / cos(lowAngle);

  double rad = 2;
  double rotNeededH = attackHigh/rad/6;
  double rotNeededL = attackLow/rad/6;

  flyWheel1 = rotNeededL;
  flyWheel2 = rotNeededL;

  lcd::print(2, "DistH: %f, theta: %f, ", robotGoal.distBetweenH, robotGoal.angleBetweenV);
}

void graphFunction(void){
  int xRes = 480;
  int yRes = 272;

  double lowAngle = 0;
  double acceleration = 7.726667;
  robot.xpos = 0;
  robot.ypos = 3;
  robot.zpos = 0;
  homeGoal.xpos = 10;
  homeGoal.ypos = 0;
  homeGoal.zpos = 0;

  double finalDone[xRes];
  for (int i = 0; i < xRes; i++){
    homeGoal.xpos = i * 60 /479;
    double attackLow = sqrt((acceleration * robotGoal.distBetweenH) /
                            fabs(2*tan(robotGoal.angleBetweenV - lowAngle)))         * cos(robotGoal.angleBetweenV) / cos(lowAngle);

    finalDone[i] = attackLow;
    delay(20);
  }


  screen::set_pen(COLOR_BLACK);
  screen::draw_rect(1,1,480,272);
  for (int i = 0; i < xRes; i++){
    screen::draw_pixel(i+10, finalDone[i]+10);
  }
  screen::draw_line(0,10,480,10);
  screen::draw_line(10,0,10,272);
}
