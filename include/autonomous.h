#ifndef __AUTONOMOUS_H__
#define __AUTONOMOUS_H__

enum autonMode_t{noAuton, flipShoot, winPoint};
enum startPos_t{near, far};
extern autonMode_t autonMode;
extern startPos_t startPos;

extern void raiseAScore(void);
extern void autonomousReal();
extern bool recoilPrevent;

#endif