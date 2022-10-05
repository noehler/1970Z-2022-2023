#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

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
};

extern chassis_t chassis;

extern void motorControl(void);

#endif