#include "main.h"
#include "backgroundFuncs.h"
#include "pros/llemu.hpp"
#include "robotConfig.h"

void initialize() {
	inertial.reset();
	//pros::lcd::initialize();

	int startTime = millis();
	while (inertial.is_calibrating() ){
		std::cout << "\nCalibrating!";
		delay(40);
		if (millis() - startTime > 3000){
			delay(50);
			master.clear();
			delay(50);
			master.print(2,1,"Calibration Failing.");
			delay(50);
			master.print(1,1,"B to ignore.");
			if (master.get_digital(DIGITAL_B)){
				break;
			}
		}
	}
	std::cout << "\nDone Calibrating!\n\n\n";

	//inertial.set_heading(180);

	/*robot.xpos = 82;
    robot.ypos = 0;
    robot.zpos = 24;

	homeGoal.xpos = 24;
	homeGoal.ypos = 48;
	homeGoal.zpos = 24;*/
	
	Task my_task(mainLoop);

	//starting main loop that handles odometry and turret angle
	//mainLoop();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	/*while (1){
		while (!master.get_digital_new_press(DIGITAL_L1)){
			std::cout << leftEncoderFB.get_value() << "\n";
			delay(20);
		}
		leftEncoderFB.reset();
	}*/
    int startTime = millis();
	while(1){
		//left normal speed and right normal speed (as in not using mechanum superpowers)
		int LNSpeed = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X);
		int RNSpeed = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X);

		//mechanum(magic) speed
		int MSpeed = -master.get_analog(ANALOG_LEFT_X);

		lfD.move(-LNSpeed + MSpeed);
		lbD.move(-LNSpeed - MSpeed);
		rfD.move(-RNSpeed - MSpeed);
		rbD.move(-RNSpeed + MSpeed);

		static int currDiffMode = 0;
		if(master.get_digital_new_press(DIGITAL_L1)){
		currDiffMode++;
		if (currDiffMode == 3){
		currDiffMode = 0;
		}
		}

		if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_B)
		 && master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_Y) && millis() - startTime > 500){
			delay(50);
			master.clear_line(1);
			delay(50);
			master.print(1, 1, "Entering dev mode");
			runLoop = false;
			devMode();
			delay(50);
			master.clear_line(1);
			delay(50);
			master.print(1, 1, "Exiting dev mode");
			runLoop = true;
			startTime = millis();
		}

	}
}
