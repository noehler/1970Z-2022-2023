#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

#include "api.h"
#include "flywheelCode.h"
#include "velTrack.h"
#include "buttons.h"
#include "devFunctions.h"

using namespace pros;

extern Controller master;

extern Motor leftFrontDrive;
extern Motor leftBackDrive;
extern Motor rightFrontDrive;
extern Motor rightBackDrive;

extern Motor flyWheel1;
extern Motor flyWheel2;
extern Motor flyWheel3;
extern Motor flyWheel4;


extern Motor turrYRot1;
extern Motor turrYRot2;

extern ADIDigitalIn shootButton;
extern ADIDigitalOut shootPiston;

extern Imu inertial;


#endif
