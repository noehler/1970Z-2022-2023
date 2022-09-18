#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

#include "api.h"
#include "flywheelCode.h"
#include "GUI.h"
#include "devFunctions.h"
#include "odometry.h"
#include "backgroundFuncs.h"

using namespace pros;

extern Controller master;

extern Motor lfD;
extern Motor lbD;
extern Motor rfD;
extern Motor rbD;

extern Motor flyWheel1;
extern Motor flyWheel2;

extern Motor diff1;
extern Motor diff2;

extern ADIDigitalOut shootPiston;
extern ADIDigitalOut elevatePiston;

extern ADIUltrasonic leftUltra;
extern ADIUltrasonic rightUltra;
extern ADIUltrasonic revUltra;

extern ADIEncoder leftEncoderFB;
extern ADIEncoder rightEncoderFB;
extern ADIEncoder encoderLR;
extern Rotation turretAngle;


extern Imu inertial;

#endif
