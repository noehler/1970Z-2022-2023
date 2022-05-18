#include "Main.h"

using namespace pros;

Controller master(pros::E_CONTROLLER_MASTER);
motor_gearset_e driveGear = E_MOTOR_GEARSET_36 ;

Motor leftFrontDrive(1, driveGear, true);
Motor leftBackDrive(3, driveGear, true);
Motor rightFrontDrive(10, driveGear, true);
Motor rightBackDrive(12, driveGear, true);

Motor flyWheel1(1, E_MOTOR_GEARSET_36, false);
Motor flyWheel2(2, E_MOTOR_GEARSET_36, true);
Motor flyWheel3(3, E_MOTOR_GEARSET_36, false);
Motor flyWheel4(4, E_MOTOR_GEARSET_36, true);

Motor turrYRot1(7, E_MOTOR_GEARSET_06, true);
Motor turrYRot2(8, E_MOTOR_GEARSET_06, true);
Imu inertial(18);

ADIDigitalIn shootButton(6);
ADIDigitalOut shootPiston(7);
