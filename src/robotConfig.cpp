#include "Main.h"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_36 ;

Motor leftFrontDrive(1, driveGear, true);
Motor leftBackDrive(3, driveGear, true);
Motor rightFrontDrive(10, driveGear, true);
Motor rightBackDrive(12, driveGear, true);

Motor flyWheel1(1, E_MOTOR_GEARSET_18, false);
Motor flyWheel2(2, E_MOTOR_GEARSET_18, true);
Motor flyWheel3(3, E_MOTOR_GEARSET_18, false);
Motor flyWheel4(4, E_MOTOR_GEARSET_18, true);


Motor turrYRot(14, E_MOTOR_GEARSET_06, true);
Motor turrZRot(15, E_MOTOR_GEARSET_06, true);
