#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/*void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}*/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
//	pros::lcd::set_text(1, "Hello PROS User!");

	//pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
	master.clear();
	delay(50);
	master.print(0, 0, "A for distance Calcs");
	delay(50);
	master.print(1, 0, "B for joyStick speed");

	while (1){
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			master.clear();
			delay(50);
			master.print(1, 0, "B to exit");
			while (true) {
				static bool goingUp = false;
				static bool timing = false;
				static int goT = 0;
				static int lastA = 0;
				static int lastD = 0;


				turretSpeed();

				lcd::print(0, "AvgTemp: %f, RPM:%f", (flyWheel1.get_temperature() + flyWheel2.get_temperature()+flyWheel3.get_actual_velocity() + flyWheel4.get_actual_velocity())/4,
				 													(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity()+flyWheel3.get_actual_velocity() + flyWheel4.get_actual_velocity())/4*49);
				//lcd::print(2, "TA: %d, TD: %d", lastA, lastD);

				//trackNums();

				if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){

					master.clear();
					delay(50);
					master.print(0, 0, "A for distance Calcs");
					delay(50);
					master.print(1, 0, "B for joyStick speed");
					lcd::clear();
					flyWheel1.brake();
					flyWheel2.brake();
					flyWheel3.brake();
					flyWheel4.brake();
					break;
				}

				delay(20);
			}
		}
		else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
			master.clear();

			delay(50);
			master.print(1, 0, "B to exit");
			while (true) {

				 int pctADD =  master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)/50;
				 static int pctTotal = 0;

				 if ((pctADD > 0 && pctTotal < 350) || (pctADD < 0 && pctTotal > -350)){
					 pctTotal += pctADD;
				 }

				 if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
					 pctTotal = 0;
				 }

				 if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){

					 master.clear();
					 delay(50);
				 	 master.print(0, 0, "A for distance Calcs ");
				 	 delay(50);
				 	 master.print(1, 0, "B for joyStick speed ");
					 lcd::clear();
					 pctTotal = 0;
					 flyWheel1.brake();
					 flyWheel2.brake();
					 flyWheel3.brake();
					 flyWheel4.brake();
					 break;
				 }

				 flyWheel1 = abs(pctTotal);
				 flyWheel2 = abs(pctTotal);
				 flyWheel3 = abs(pctTotal);
				 flyWheel4 = abs(pctTotal);

				 lcd::print(0, "AvgTemp: %f, RPM:%f", (flyWheel1.get_temperature() + flyWheel2.get_temperature()+flyWheel3.get_actual_velocity() + flyWheel4.get_actual_velocity())/4,
				  													(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity()+flyWheel3.get_actual_velocity() + flyWheel4.get_actual_velocity())/4*49);

				 delay(40);
			}
		}
		delay(20);
	}
}
