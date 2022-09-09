// made on July 28, 2022 by Nathaniel Oehler

#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

class position_t {
  public:
    double xpos, ypos, zpos, width, length;
    double angle;
    double velocity, xVelocity, yVelocity;
};

class robotGoalRelatives {
  public:
    //storing the absolute and relative horizontal angle between goal and robot
    double angleBetweenHorABS;
    double angleBetweenHorREL;
    double dx, dy,dz;

    //storing angle between turret and goal
    double angleBetweenV;

    //storing the distance the frizbee will travel horizontally
    double distBetweenH;
};

extern robotGoalRelatives robotGoal;
extern position_t robot;
extern position_t homeGoal;

extern void basicOdometry(void);

#endif