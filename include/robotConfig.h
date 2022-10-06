#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

#include "api.h"
#include "flywheelCode.h"
#include "GUI.h"
#include "devFunctions.h"
#include "odometry.h"
#include "autonomous.h"
#include "backgroundFuncs.h"
#include "motorControl.h"

using namespace pros;

extern Controller master;
extern Controller sidecar;

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
extern Rotation turretEncoder;
extern Imu inertialTurret;


extern ADIAnalogIn upLoaded;
extern ADIAnalogIn deckLoaded;
extern ADIAnalogIn holeLoaded;

extern Imu inertial;

extern Optical opticalSensor;

#endif
