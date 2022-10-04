#include "main.h"
#include "robotConfig.h"
#include <cmath>

void initialize() {
	guiInit();
	robotGoal.dx = 10;
	robotGoal.dy = 10;
	robotGoal.dz = 10;

	inertial.reset();
	inertialTurret.reset();

	int startTime = millis();
	while (inertial.is_calibrating()  || inertialTurret.is_calibrating()){
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


	Task sLoop(startLoop);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	// i want to go to world...
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

		delay(20);

	}
}
