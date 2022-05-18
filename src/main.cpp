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
	//pros::lcd::initialize();
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

void turretControl_fn(void){
	while(1){
		turrYRot1 = -master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)/3;
		turrYRot2 = -master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)/3;

		if(shootButton.get_value() || master.get_digital(E_CONTROLLER_DIGITAL_A) ){
			shootPiston.set_value(true);
			delay(100);
			shootPiston.set_value(false);
		}
		/*if (abs(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)/3) < 2){
			turrYRot1.set_brake_mode(MOTOR_BRAKE_HOLD);
			turrYRot2.set_brake_mode(MOTOR_BRAKE_HOLD);
			turrYRot1.brake();
			turrYRot2.brake();
		}*/
		delay(20);
	}
}

void opcontrol() {
	inertial.reset();

	while(inertial.is_calibrating()){
		std::cout << "\nCalibrating!";
		delay(40);
	}
	std::cout << "\nDone Calibrating!\n\n\n";

	inertial.set_heading(180);

  turrYRot1.move_absolute(300, 127);
  turrYRot1.move_absolute(300, 127);
	/*Task turretControl(turretControl_fn);
	flyWheel1 = abs(127);
	flyWheel2 = abs(127);
	flyWheel3 = abs(127);
	flyWheel4 = abs(127);
	readYRot();*/

	//numTrain();
	//turretControl();
	master.clear();
	delay(50);
	master.print(0, 0, "A for distance Calcs");
	delay(50);
	master.print(1, 0, "B for joyStick speed");
	while (1){
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			delay(50);
			master.clear();
			delay(50);
			master.print(1, 0, "B to exit");


			//graphFunction();
			while (true) {

				static bool robCoorGot = false;
				static bool goingUp = false;
				static bool timing = false;
				static int goT = 0;
				static int lastA = 0;
				static int lastD = 0;

				//getting info from laptop
				if (!robCoorGot)
				{
					robot.xpos = getNum("Robot X: ");
					robot.ypos = getNum("Robot Y: ");
					robot.zpos = getNum("Robot Z: ");

					robCoorGot = true;
				}


				homeGoal.xpos = getNum("Target X: ");
				homeGoal.ypos = getNum("Target Y: ");
				homeGoal.zpos = getNum("Target Z: ");

				updateTurretAngle();


				flyWheel1 = 100;
				flyWheel2 = 100;
				flyWheel3 = 100;
				flyWheel4 = 100;


				//turretSpeed();

				while((fabs(flyWheel1.get_actual_velocity()) < abs(flyWheel1.get_target_velocity()) *.9)){
					std::cout << "\n" <<inertial.get_heading()  << ":" << degreeHope;
 					//std::cout << "\n" <<fabs(flyWheel1.get_actual_velocity()) << " : " << abs(flyWheel1.get_target_velocity())*.9 << "\n" <<flyWheel1.get_temperature();
					delay(40);
				}
				std::cout << "\n\nFire!\n\n";
				while(!shootButton.get_new_press() && !master.get_digital(E_CONTROLLER_DIGITAL_A) ){
					std::cout << "\n" <<inertial.get_heading()  << ":" << degreeHope;
					//std::cout << "\n" <<fabs(flyWheel1.get_actual_velocity()) << " : " << abs(flyWheel1.get_target_velocity())*.9;
					delay(40);
				}
				shootPiston.set_value(true);
				delay(100);
				shootPiston.set_value(false);

				//trackNums();

				delay(500);

				flyWheel1.brake();
				flyWheel2.brake();
				flyWheel3.brake();
				flyWheel4.brake();

				/*flyWheel1.brake();
				flyWheel2.brake();
				flyWheel3.brake();
				flyWheel4.brake();*/

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

				 if ((pctADD > 0 && pctTotal < 150) || (pctADD < 0 && pctTotal > -150)){
					 pctTotal += pctADD;
				 }

				 if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
					 pctTotal = 0;
				 }

				 shootPiston.set_value(shootButton.get_value());

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
