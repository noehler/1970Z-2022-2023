#ifndef __DEVFUNCS_H__
#define __DEVFUNCS_H__
#include "api.h"
extern void flyTuner(pros::Motor flyWheel1, pros::Motor flyWheel2, int loopDelay);
extern void moveToCalibrate(double &leftSpd, double &rightSpd);
extern void calibrateDriveTo(double goalDist);
extern void driveTuner(void);
extern void bezMake(void);
class PID_t{
    public:
        double p, i, d, p2, i2,d2;
};
#endif