#include "main.h"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_06 ;

Motor lfD(1, driveGear, true);
Motor lbD(2, driveGear, true);
Motor rfD(7, driveGear, true);
Motor rbD(4, driveGear, true);

Motor flyWheel1(9, E_MOTOR_GEARSET_36, false);
Motor flyWheel2(10, E_MOTOR_GEARSET_36, true);

Motor diff1(5, E_MOTOR_GEARSET_06, true);
Motor diff2(6, E_MOTOR_GEARSET_06, true);
Imu inertial(11);
Rotation turretAngle(8);


ADIDigitalOut shootPiston({{22,'A'}});
ADIDigitalOut elevatePiston({{22,'B'}});

ADIUltrasonic leftUltra({{22,'A', 'B'}});
ADIUltrasonic rightUltra({{22,'B','B'}});
ADIUltrasonic revUltra({{22,'A','B'}});

ADIEncoder leftEncoderFB({{20,'C','D'}, true});
ADIEncoder rightEncoderFB({{22,'C', 'D'}});
ADIEncoder encoderLR({{20,'A','B'}});
