#ifndef __AUTONSETUP_H__
#define __AUTONSETUP_H__

enum autonTypes_t{
    winPointFar, winPointClose, noAuton, skillsAuton
};

extern double distto(double x, double y);
extern void moveto(void *mc, double xTo, double yTo, double speedLimit = 100, double tolerance = 5, double errtheta = 5, int forward = 1);
extern double pickPos(double posInput, int run);
extern void shootdisks(void *mc, int overallStartTime, bool calibrapePos = false, int number = 4);
extern void intake(void *mc);
extern void movevoltage(void *mc, double L, double R);
extern void rotateto(void *mc,double ang, double range = 3);
extern void intakeWaitForDiscs(void *mc, int maxTime, int overallStartTime, int goalAmt = 3);
extern void waitRotate(void *mc, int maxTime, int overallStartTime);

extern autonTypes_t autonType;
extern int color;

#endif