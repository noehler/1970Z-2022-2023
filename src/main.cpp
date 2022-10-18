#include "main.h"
#include "odometry.h"
#include "robotConfig.h"
#include <cmath>

double mod(double base, double var){
  while (base<var){//e.g. 361mod360 = 1
    var-=base;
  }
  while (var<0){ // e.g. -361mod 360 = 359
    var+=base;
  }
  return var;
}
void initialize() {
  while (1){
    vision_object_s_t codeTemp = turVisionL.get_by_code(0, REDGOAL);
    std::cout << "(" << codeTemp.x_middle_coord << ", " << codeTemp.y_middle_coord << ")\n";
    delay(20);
  }
  boomShackalacka.set_value(false);
	inertial.reset();
	inertialTurret.reset();
	guiInit();

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
	inertial.set_heading(0);
	inertialTurret.set_heading(0);
	delay(50);
	master.clear();
	delay(50);
	master.print(1,1,"Calibration Success.");
	robot.xVelocity=0;
	robot.yVelocity = 0;
	robot.wVelocity = 0;
	robot.velocity = 0;
	robot.chaIntAng = 270;
	robot.TurintAng = 90;

	robot.xpos = 144-robot.width/2;
	robot.ypos = 72;
	robot.zpos = 12.2;

	homeGoal.xpos = 20;
	homeGoal.ypos = 124;
	homeGoal.zpos = 25;
  Task turrC(motorControl);
  Task varUP(updateInfoLoop);
	Task sLoop(startLoop);
}

void disabled() {
	while(1){
		controller2();
	}
}

void competition_initialize() {}
void autonomous(){
	autonomousReal();
}

void opcontrol() {
	// i want to go to world...
	while(1){/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//left normal speed and right normal speed (as in not using mechanum superpowers)
		chassis.driveTrain.leftSpd = -master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X);
		chassis.driveTrain.rightSpd = -master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X);

		//mechanum(magic) speed
		chassis.driveTrain.mechSpd = -master.get_analog(ANALOG_LEFT_X);

		//std::cout << "\nturenc:"<<double(turretEncoder.get_position())/2158.3333<<"chHeading:"<<-inertial.get_heading();
		liftConrol();
    if (sidecar.is_connected()){
		    controller2();
    }
    if (usd::is_installed()){
      outPosSDCARD();
    }

		devCheck();

		//have a 150 msec delay because of updating controller screen
		//delay(20);

	}
}
