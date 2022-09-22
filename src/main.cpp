#include "main.h"
#include "robotConfig.h"
#include <cmath>

void initialize() {
	/*while(1){
		std::cout << "\nLENC:" << leftEncoderFB.get_value() << ", RENC:" << rightEncoderFB.get_value() << ", SENC:" << encoderLR.get_value();
		delay(30);
	}*/
	guiInit();
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
	runLoop = false;
	while(1){
		std::cout<<"\nentering turrTest";
		static int spd = 0;
		if (master.get_digital(DIGITAL_R1)){
			spd+=1;
		}
		else if (master.get_digital(DIGITAL_R2)){
			spd-=1;
		}

		delay(50);
		master.clear();
		delay(50);
		master.print(0, 1, "Goal pct: %d", spd);
		flyWheel1 = spd;
		flyWheel2 = spd;
		shootPiston.set_value(master.get_digital(DIGITAL_A));
		if (master.get_digital_new_press(DIGITAL_B)){
			static bool lifting = false;
			lifting = !lifting;
			elevatePiston.set_value(lifting);
		}
	}

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
