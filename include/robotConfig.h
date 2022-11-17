#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

//will always end up being false but makes edditor realize that api.h is seen
#ifndef _PROS_MAIN_H_
#include "api.h"
#endif
using namespace pros;


extern Controller master;
extern Controller sidecar;

extern ADIDigitalOut boomShackalacka;

extern ADIDigitalOut shootPiston;
extern ADIDigitalOut elevatePiston;

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

extern Vision turVisionL;
extern Vision turVisionR;
extern vision_signature_s_t REDGOAL;
extern vision_signature_s_t BLUEGOAL;

#endif