#include "main.h"
#include "pros/adi.hpp"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_06 ;

Motor lfD(1, driveGear, true);
Motor lbD(2, driveGear, true);
Motor rfD(7, driveGear, false);
Motor rbD(4, driveGear, false);

Motor flyWheel1(18, E_MOTOR_GEARSET_36, true);
Motor flyWheel2(19, E_MOTOR_GEARSET_36, false);

Motor diff1(5, E_MOTOR_GEARSET_06, true);
Motor diff2(6, E_MOTOR_GEARSET_06, true);
Imu inertial(11);
Rotation turretAngle(8);


ADIDigitalOut shootPiston({{22,'G'}});
ADIDigitalOut elevatePiston({{22,'H'}});

ADIAnalogIn upLoaded({22,'F'});
ADIAnalogIn deckLoaded({20,'H'});
ADIAnalogIn holeLoaded({22,'E'});

ADIUltrasonic leftUltra({{22,'A', 'B'}});
ADIUltrasonic rightUltra({{22,'B','B'}});
ADIUltrasonic revUltra({{22,'A','B'}});

ADIEncoder leftEncoderFB({{20,'C','D'}, true});
ADIEncoder rightEncoderFB({{22,'C', 'D'}});
ADIEncoder encoderLR({{20,'A','B'}});
