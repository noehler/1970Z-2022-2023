#ifndef __AUTONSETUP_H__
#define __AUTONSETUP_H__

enum autonTypes_t{
    winPointFar, winPointClose, noAuton, skillsAuton, winPointBoth
};

extern double distto(double x, double y);
extern void moveto(void *mc, double xTo, double yTo, double speedLimit = 100, double tolerance = 5, double errtheta = 5, int forward = 1);
extern void shootdisks(void *mc, int overallStartTime, bool calibrapePos = false, int number = 4, int overallMaxTime = 55000, int maxTime = 5000);
extern void intake(void *mc);
extern void movevoltage(void *mc, double L, double R);
extern void rotateto(void *mc,double ang, double range = 3);
extern void intakeWaitForDiscs(void *mc, int maxTime, int overallStartTime, int goalAmt = 3, int overallMaxTime = 55000);
extern void waitRotate(void *mc, int maxTime, int overallStartTime, int overallMaxTime = 55000);
extern void waitPosTime(void *mc, int maxTime, int overallStartTime, int overallMaxTime = 55000);
extern void AutonSelector(void);


extern bool updateScreen[2];

extern autonTypes_t autonType;
extern int color;

#endif