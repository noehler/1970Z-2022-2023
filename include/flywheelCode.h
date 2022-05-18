#ifndef __FLYWHEELCODE_H__
#define __FLYWHEELCODE_H__

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
extern void updateTurretAngle(void);
extern void turretSpeed(void);
extern void angleVertBetween(void);
extern double angularVelocityCalc(double, double, double, double);
extern double degreeHope;

#endif
