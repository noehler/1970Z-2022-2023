#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "pros/colors.h"
#include "pros/motors.h"

class drivespeeds_t {
  public:
    double leftSpd, rightSpd, mechSpd;
    bool running = true;
};

class piston_t {
  public:
    bool shoot, elevate;
};


class chassis_t {
  public:
  
    drivespeeds_t driveTrain;

    piston_t pistons;

    double flyRPM;

    bool isSpinner = false;

    //0 is blue, 1 is red
    int teamColor = 1;

    //tV values and what they mean
    //0 is stopped
    //1 is reversed
    //2 is fwd
    int intakeRunning;
};

class moveToInfoExternal_t{
  public:
    double moveToxpos, moveToypos, targetHeading, speed_limit=100, errtheta=5;
    int moveToforwardToggle = 1, Stop_type = 2;
    double tolerance=5;
    bool resetMoveTo;
};

extern moveToInfoExternal_t move;
extern double diffFlyWheelW;
extern chassis_t chassis;

extern void motorControl(void);

#endif