#include "Main.h"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_36 ;

Motor leftFrontDrive(1, driveGear, true);
Motor leftBackDrive(3, driveGear, true);
Motor rightFrontDrive(10, driveGear, true);
Motor rightBackDrive(12, driveGear, true);

Motor flyWheel(20, E_MOTOR_GEARSET_18, true);
Motor turrYRot(20, E_MOTOR_GEARSET_36, true);
Motor turrZRot(20, E_MOTOR_GEARSET_36, true);;
