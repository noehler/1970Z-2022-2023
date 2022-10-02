#include "main.h"
#include "pros/adi.hpp"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_06 ;

Motor lfD(1, driveGear, true);
Motor lbD(2, driveGear, true);
Motor rfD(7, driveGear, false);
Motor rbD(4, driveGear, false);

Motor flyWheel1(11, E_MOTOR_GEARSET_36, false);
Motor flyWheel2(14, E_MOTOR_GEARSET_36, true);

Motor diff1(5, E_MOTOR_GEARSET_06, true);
Motor diff2(6, E_MOTOR_GEARSET_06, true);
Imu inertial(8);
Imu inertialTurret(12);
Rotation turretAngle(10);


ADIDigitalOut shootPiston({{22,'B'}});
ADIDigitalOut elevatePiston({{9,'G'}});

ADIAnalogIn upLoaded({22,'F'});
ADIAnalogIn deckLoaded({9,'H'});
ADIAnalogIn holeLoaded({22,'E'});

ADIUltrasonic leftUltra({{22,'E', 'F'}});
ADIUltrasonic rightUltra({{22,'G','H'}});
ADIUltrasonic revUltra({{22,'C','D'}});

ADIEncoder leftEncoderFB({{9,'C','D'}, true});
ADIEncoder rightEncoderFB({{9,'E', 'F'},true });
ADIEncoder encoderLR({{9,'A','B'}});
