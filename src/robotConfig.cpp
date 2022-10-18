#include "main.h"
#include "pros/adi.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
Controller sidecar(pros::E_CONTROLLER_PARTNER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_06 ;

Motor lfD(1, driveGear, true);
Motor lbD(2, driveGear, true);
Motor rfD(7, driveGear, false);
Motor rbD(4, driveGear, false);

Motor flyWheel1(11, E_MOTOR_GEARSET_06, false);
Motor flyWheel2(13, E_MOTOR_GEARSET_06, true);

Motor diff1(5, E_MOTOR_GEARSET_06, true);
Motor diff2(6, E_MOTOR_GEARSET_06, true);
Imu inertial(8);
Imu inertialTurret(12);
Rotation turretEncoder(10);
Optical opticalSensor(18);

ADIDigitalOut shootPiston({{22,'B'}});
ADIDigitalOut elevatePiston({{9,'G'}});
ADIDigitalOut boomShackalacka({{22,'C'}});

ADIAnalogIn upLoaded({22,'F'});
ADIAnalogIn deckLoaded({9,'H'});
ADIAnalogIn holeLoaded({22,'E'});

//ADIUltrasonic leftUltra({{22,'E', 'F'}});
//ADIUltrasonic rightUltra({{22,'G','H'}});
//ADIUltrasonic revUltra({{22,'C','D'}});

ADIEncoder leftEncoderFB({{9,'C','D'}, true});
ADIEncoder rightEncoderFB({{9,'E', 'F'},true });
ADIEncoder encoderLR({{9,'A','B'}});

Vision turVisionL ( 20, E_VISION_ZERO_CENTER);
Vision turVisionR ( 19, E_VISION_ZERO_CENTER);
vision_color_code_t REDGOAL = turVisionL.create_color_code(1, 1483, 5187, 3335, -1139, 283, -428, 1.400, 0);
vision_color_code_t BLUEGOAL = turVisionL.create_color_code(2, -1671, 1, -834, 2025, 4413, 3220, 2.000, 0);
