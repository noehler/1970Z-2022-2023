#include "main.h"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
Controller sidecar(pros::E_CONTROLLER_PARTNER);



ADIEncoder leftEncoderFB({{9,'C','D'}, true});
ADIEncoder rightEncoderFB({{9,'E', 'F'},true });
ADIEncoder encoderLR({{9,'A','B'}});

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

Vision turVisionL ( 15);
Vision turVisionR ( 19);
vision_signature_s_t REDGOAL = Vision::signature_from_utility(1, 1483, 5187, 3335, -1139, 283, -428, 1.400, 0);
vision_signature_s_t BLUEGOAL = Vision::signature_from_utility(2, -1671, 1, -834, 2025, 4413, 3220, 2.000, 0);