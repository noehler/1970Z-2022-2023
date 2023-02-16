#include "main.h"
#include "Autons/autonSetup.h"
#include "devFuncs.h"
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
	Task gps_Task(GPS_Wrapper, (void*) &sensing, "GPS Task");
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
	autonType = winPointClose;
	if (autonType == winPointClose){//close win Point auton
		motorControl_t motorControl;
		motorControl.setpistons();

		Task fly_Task(fly_ControllerWrapper, (void*) &motorControl, "My Flywheel Speed Controller Task");

		sensing.goalSpeed = 179;
		delay(3000);
		//goalAngle = 0;
		motorControl.raiseAScore(1);
		
		motorControl.driveToRoller();
		//fly_Task.suspend();

	}
	else if (autonType == winPointFar){//far win Point auton
		/*chaIntAng = 90;
		motorControl_t motorControl;
		motorControl.setpistons();

		Task fly_Task(fly_ControllerWrapper, (void*) &motorControl, "My Flywheel Speed Controller Task");

		sensing.goalSpeed = 200;
		goalAngle = 0;
		delay(5000);
		motorControl.raiseAScore(1);

		motorControl.rotateTo(0);

		motorControl.driveDist(-15);
		
		motorControl.rotateTo(70);
		delay(1000);
		
		motorControl.driveToRoller();
		*/
	}
	else if (autonType == noAuton){
		
	}
	else {	
		//skillsAutonomous();
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
	Motor turretMotor(6, E_MOTOR_GEARSET_06, true);
	PID_t PID;
	PID.p = 0.1;
	PID.i = .00001;
	PID.d = 0.4;
	turretTuner(135, PID, 10000, 300, (void*) &turretMotor);

	chaIntAng = 0;
	motorControl_t motorControl;
	Task drive_Task(drive_ControllerWrapper, (void*) &motorControl, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &motorControl, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &motorControl, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
	
	while (1){
		static bool started = false;
		static bool autoAim = false;
		if (master.get_digital_new_press(DIGITAL_Y)){
			autoAim = !autoAim;
		}
		if (autoAim == false){
			sensing.SSOSTTT_bool = false;
			goalAngle = sensing.robot.angle+180;
			sensing.goalSpeed = 180;
			started = false;
		}
		else{
			if (started == false){
				started = true;
			}
		}
		logValue("time", millis(),0);

		logValue("xTot", sensing.robot.xpos, 1);
		logValue("yTot", sensing.robot.ypos, 2);
		
		logValue("xGPS", sensing.robot.GPSxpos, 3);
		logValue("yGPS", sensing.robot.GPSypos, 4);

		logValue("xOdom", sensing.robot.odoxpos, 5);
		logValue("yOdom", sensing.robot.odoypos, 6);

		logValue("robot angle", sensing.robot.angle, 7);
		logValue("turret angle", sensing.robot.turAng, 8);
		outValsSDCard();

		delay(20);
	}
}
