#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

class drivespeeds_t {
  public:
    double leftSpd, rightSpd, mechSpd;
};



class chassis_t {
  public:
    drivespeeds_t driveTrain;

};


extern void motorControl(void);

#endif