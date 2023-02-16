#ifndef __DEVFUNCS_H__
#define __DEVFUNCS_H__

extern void moveToCalibrate(double &leftSpd, double &rightSpd);
extern void calibrateDriveTo(double goalDist);
class PID_t{
    public:
        double p, i, d, p2, i2,d2;
};
extern void turretTuner(double goal, PID_t PIDU, int timeToTest, int delayTiming, void* motor);
#endif