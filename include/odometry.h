// made on July 28, 2022 by Nathaniel Oehler

#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

class position_t {
  public:
    double xpos, ypos, zpos;
    double angle;
    double velocity;
};

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

extern robotGoalRelatives robotGoal;
extern position_t robot;
extern position_t homeGoal;


#endif