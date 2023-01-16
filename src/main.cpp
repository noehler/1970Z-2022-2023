#include "main.h"
#include "Autons/autonSetup.h"
#include "motorControl.h"
#include "pros/motors.hpp"
#include "robotConfig.h"
#include "sdLogging.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	setupScreen();
	motorControl_t mc;
	mc.setpistons();
	sensing.setUp();
	Task odometry_Task(odometry_Wrapper, (void*) &sensing, "Odometry Task");
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	if (autonType == winPointClose || autonType == skillsAuton){//close win Point auton
		motorControl_t motorControl;
		motorControl.setpistons();

		Task fly_Task(fly_ControllerWrapper, (void*) &motorControl, "My Flywheel Speed Controller Task");

		sensing.goalSpeed = 305;
		goalAngle = 0;

		delay(5000);
		motorControl.raiseAScore();

		motorControl.driveToRoller();
		fly_Task.suspend();

	}
	else if (autonType == winPointFar){//far win Point auton
		motorControl_t motorControl;
		motorControl.setpistons();

		Task fly_Task(fly_ControllerWrapper, (void*) &motorControl, "My Flywheel Speed Controller Task");

		sensing.goalSpeed = 305;
		goalAngle = 0;

		delay(5000);
		motorControl.raiseAScore();
		
		Task drive_Task(drive_ControllerWrapper, (void*) &motorControl, "My Driver Controller Task");
		motorControl.move.moveToxpos = 108;
		motorControl.move.moveToypos = 132;
		motorControl.move.tolerance = 2;
		motorControl.waitPosTime(3000);

		motorControl.move.moveToxpos = 108;
		motorControl.move.moveToypos = 140;
		motorControl.waitPosTime(3000);
		drive_Task.suspend();
		
		motorControl.driveToRoller();
	}
	else if (autonType == noAuton){
		
	}

	if (autonType == skillsAuton){
		motorControl_t mc;
		Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
		mc.move.tolerance = 2;

		mc.waitPosTime(3000);

		mc.explode();
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	sensing.goalSpeed = 300;
	motorControl_t motorControl;

	goalAngle = sensing.robot.angle + 180;
	while (goalAngle > 360){
		goalAngle -= 360;
	}
	while (goalAngle < 0){
		goalAngle += 360;
	}
	
	//Task vision_Task(VT_Wrapper, (void*) &motorControl, "My vision Controller Task");
	Task drive_Task(drive_ControllerWrapper, (void*) &motorControl, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &motorControl, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &motorControl, "My Flywheel Speed Controller Task");

	while (true) {
		sensing.goalSpeed = 300;
		goalAngle = sensing.robot.angle + 180;
		while (goalAngle > 360){
			goalAngle -= 360;
		}
		while (goalAngle < 0){
			goalAngle += 360;
		}
		//motorControl.speedToggle();		
		pros::delay(20);
	}
}
