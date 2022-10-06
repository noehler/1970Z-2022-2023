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


    homeGoal.xpos = 120;
    homeGoal.ypos = 120;
    homeGoal.zpos = 25;
    
    robot.xpos = 0;
    robot.ypos = 0;
    robot.zpos = 12.2;
    
	move.moveToxpos = 30;
	move.moveToypos = 0;

    robot.xVelocity = 0;
    robot.yVelocity = 0;

    setAngle(turret, 180);

    Task turrC(motorControl);
    Task varUP(updateInfoLoop);
	Task sLoop(startLoop);
}

void disabled() {}

void competition_initialize() {}

void opcontrol() {
	// i want to go to world...
	while(1){
		//left normal speed and right normal speed (as in not using mechanum superpowers)
		chassis.driveTrain.leftSpd = -master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X);
		chassis.driveTrain.rightSpd = -master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X);

		//mechanum(magic) speed
		chassis.driveTrain.mechSpd = -master.get_analog(ANALOG_LEFT_X);
		
		liftConrol();
		
		devCheck();

		delay(20);

	}
}
