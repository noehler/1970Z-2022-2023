#include "main.h"
#include "backgroundFuncs.h"
#include "devFunctions.h"
#include "pros/llemu.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"

void initialize() {
	/*while(1){
		std::cout << "\nLENC:" << leftEncoderFB.get_value() << ", RENC:" << rightEncoderFB.get_value() << ", SENC:" << encoderLR.get_value();
		delay(30);
	}*/
	robotGoal.dx = 10;
	robotGoal.dy = 10;
	robotGoal.dz = 10;

	inertial.reset();

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
	delay(50);
	master.clear();
	delay(50);
	master.print(1,1,"Calibration Success.");
	std::cout << "\nDone Calibrating!\n\n\n";
	Task sLoop(startLoop);
	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	/*runLoop = false;
	leftEncoderFB.reset();
	rightEncoderFB.reset();
	encoderLR.reset();
	while(1){
		while(!master.get_digital_new_press(DIGITAL_A)){
			delay(20);
		}
		std::cout << "\n" << func();
		delay(20);
	}*/
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

		liftConrol();

		devCheck();

	}
}
