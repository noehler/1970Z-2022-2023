// made on July 28, 2022 by Nathaniel Oehler

#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

class position_t {
  public:
    double xpos, ypos, zpos, width = 11.5, length = 6;
    double xposodom,yposodom,xposvision,yposvision;
    double angle,turAng;
    double chaIntAng,TurintAng;
    double velocity, xVelocity, yVelocity, wVelocity, turvelocity;
};

class robotGoalRelatives {
  public:
    //storing the absolute and relative horizontal angle between goal and robot
    double angleBetweenHorABS;
    double angleBetweenHorREL;
    double dx, dy,dz;
};

extern robotGoalRelatives robotGoal;
extern position_t robot;
extern position_t homeGoal;
#include <iostream>
#include "string.h"

extern void enc_odometry(void);
extern void imu_odometry(void);
extern void mot_odometry(void);
extern void turAngupdate(void);
extern void visionOdom(void);

#endif