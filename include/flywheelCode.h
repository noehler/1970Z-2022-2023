#ifndef __FLYWHEELCODE_H__
#define __FLYWHEELCODE_H__

class position_t {
  public:
    double xpos, ypos, zpos;
    double angle;
    double velocity;
};

extern position_t robot;
extern position_t homeGoal;

#endif
