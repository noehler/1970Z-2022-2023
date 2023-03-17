#include "main.h"
#include "Autons/autonSetup.h"

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
  sensing.Init();

  Task odometry_Task(odometry_Wrapper, (void *)&sensing, "Odometry Task");
  Task outPutting_task(outValsSDCard, "Outputting Task");
  Task gps_Task(GPS_Wrapper, (void *)&sensing, "GPS Task");
  Task AutonSelector_Task(AutonSelector, "Auton Selector Task");
  sensing.set_status(37,18,270,100, 0);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
	switch(autonType){
		case winPointClose: // close win Point auton
    		closeWinPoint();
			break;
		case winPointFar://    far win Point auton
    		farWinPoint();                        
			break;
		case winPointBoth://   both roller win point
    		winPointAuton();
			break;
		case skillsAuton://	   skills auton
    		skillsAutonomous();
			break;
		case noAuton://   	   noAuton
			break;
		
	}
	while(1){
		delay(20);
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
bool vibrate = false;
void vibrateController(void){
  while(1){
    while(!vibrate){
      delay(20);
    }
    vibrate = false;
    master.rumble(".");
    delay(50);
  }
}
void opcontrol() {

	//starting up tasks neccessary for driving
	motorControl_t motorControl;

	Task drive_Task(drive_ControllerWrapper, (void *)&motorControl,
					"My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void *)&motorControl,
							"Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void *)&motorControl,
					"My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void *)&sensing, "turret angle Task");
	Task vibrationTask(vibrateController,"My Vibrator Controller Task");
  
	while (1) {  

		//adjusts goal speed for single vs double shot
		if (master.get_digital_new_press(DIGITAL_A)){
			motorControl.discCountChoice = 2;
		}
		if (master.get_digital_new_press(DIGITAL_B)){
			motorControl.discCountChoice = 1;
		}

		//adjusts goal angle to shoot at goal or able to intake
		sensing.SSOSTTT_bool = true;
		static bool autoAim = false;
		static int aimSpot = 0;
		if (master.get_digital_new_press(DIGITAL_UP)) {
			autoAim = !autoAim;
		}
		if (autoAim == false) {
			sensing.robot.turretLock = true;
			goalAngle = sensing.robot.angle + 180;
		} else {
			sensing.robot.turretLock = false;
		}

		//code to alert driver when angle and speed of turret are correct
		static bool turretGood = false;
		if(fabs(motorControl.diffFlyWheelW) <35 && motorControl.angdiff < 3 && sensing.robot.turretLock == false){
			if (turretGood == false){
				vibrate = true;
			}
			turretGood = true;
		}
		else{
			turretGood = false;
		}

		//code to change goal shooting position
		if (color != 2){
			if(master.get_digital(DIGITAL_LEFT)){
				//our goal
				sensing.goal.xpos = 20;
				sensing.goal.ypos = 124;
				sensing.goal.zpos = 30;
			}
			else if (master.get_digital(DIGITAL_RIGHT)){
				//middle of field
				//for expansion
				sensing.goal.xpos = 72;
				sensing.goal.ypos = 72;
			}
		}
		else{
			//automatic calculation for skills
			if (sensing.robot.xpos > sensing.robot.ypos){
				sensing.goal.xpos = 124;
				sensing.goal.ypos = 20;
				sensing.goal.zpos = 30;
			}
			else{
				sensing.goal.xpos = 20;
				sensing.goal.ypos = 124;
				sensing.goal.zpos = 30;
			}
		}
		
		//GPS Calibration
		static int loop = 0;
		static double xGPSintegral = 0;
		static double yGPSintegral = 0;
		static bool calculatePos = false;
		if (master.get_digital(DIGITAL_Y)){
			xGPSintegral +=sensing.robot.GPSxpos;
			yGPSintegral +=sensing.robot.GPSypos;
			loop++;
			calculatePos = true;
		}
		else{
			if (calculatePos && loop > 50){
				sensing.robot.xpos = xGPSintegral/loop;
				sensing.robot.ypos = yGPSintegral/loop;

				xGPSintegral = 0;
				yGPSintegral = 0;
				loop = 0;
				calculatePos = 0;
			}
			else{
				xGPSintegral = 0;
				yGPSintegral = 0;
				calculatePos = 0;
				loop = 0;
			}
		}
		
		delay(20);
	}
}
