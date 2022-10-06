#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "pros/motors.h"

class drivespeeds_t {
  public:
    double leftSpd, rightSpd, mechSpd;
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

    bool isSpinner = true;

    //0 is blue, 1 is red
    bool teamColor = 0;

    //tV values and what they mean
    //0 is stopped
    //1 is reversed
    //2 is fwd
    int intakeRunning;
};
class moveToInfo_t{
  public:
    double moveToxpos=0, moveToypos=0, targetHeading, ets=0, speed_limit=100, errtheta=30;
    double dist = 0;          // change of position
    double distR = 0;         // chagne of right postion
    double distL = 0;         // change of left position
    double PIDSS = 0;         // PID turning speed
    double PIDFW = 0;         // PID moveforward speed
    double PIDSpeedL = 0;     // PID leftside speed
    double PIDSpeedR = 0;     // PID rightside speed
    double prevPIDFW = 0;     // PID moveforward speed at t = -1
    double prevPIDSS = 0;     // PID turning speed at t = -1
    double PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
    double PIDSSFLAT = 0;   
    int moveToforwardToggle = 1, Stop_type = 2;
    bool reset = true;
};

extern moveToInfo_t move;

extern chassis_t chassis;

extern void motorControl(void);

#endif