#ifndef __DEVFUNCTIONS_H__
#define __DEVFUNCTIONS_H__

#include "iostream"

extern char outNames[20][50];
extern double outVals[20];

enum angleType{degrees, radians};
enum objectType{base, turret};

extern double getNum(std::string);

extern void graphFunction(void);
extern void alert(std::string message);
extern void numTrain(void);
extern void readYRot(void);
extern void calcRadius(void);

extern void trackNums(void);
extern void devCheck(void);
extern void devMode(void);

void logVals(std::string name = "reset",double value = 0);
extern void setAngle(objectType, int);
extern void outPosSDCARD(void);
extern void startRecord(void);


#endif
