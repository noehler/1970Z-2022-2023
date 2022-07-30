#include "main.h"

void initialize() {
	inertial.reset();

	while(inertial.is_calibrating()){
		std::cout << "\nCalibrating!";
		delay(40);
	}
	std::cout << "\nDone Calibrating!\n\n\n";

	inertial.set_heading(180);

	//starting main loop that handles odometry and turret angle
	//mainLoop();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

	while(1){
		//left normal speed and right normal speed (as in not using mechanum superpowers)
		int LNSpeed = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X);
		int RNSpeed = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X);

		//mechanum(magic) speed
		int MSpeed = master.get_analog(ANALOG_LEFT_X);

		lfD.move(LNSpeed + MSpeed);
		lbD.move(LNSpeed - MSpeed);
		rfD.move(RNSpeed - MSpeed);
		rbD.move(RNSpeed + MSpeed);

		static int currDiffMode = 0;
		if(master.get_digital_new_press(DIGITAL_L1)){
		currDiffMode++;
		if (currDiffMode == 3){
		currDiffMode = 0;
		}
		}

		if (currDiffMode == 0){
		diff1.brake();
		diff2.brake();
		}
		else if (currDiffMode == 1){
		diff1.move(127);
		diff2.move(127);
		}
		else{
		diff1.move(127);
		diff2.move(-127);
		}

	}	

}
