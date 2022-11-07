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
void initialize(){
  	opticalSensor.set_led_pwm(100);
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

	
	PID.driveFR.p = 1;
	PID.driveFR.i = 0;
	PID.driveFR.d = 3;

	PID.turret.p = .8889;
	PID.turret.i = .236;
	PID.turret.d = 1.2818;

	robot.xVelocity=0;
	robot.yVelocity = 0;
	robot.wVelocity = 0;
	robot.velocity = 0;
	robot.chaIntAng = 270;
	robot.TurintAng = 90;

	robot.xpos = 29.5;
	robot.ypos = 18.5;
	robot.zpos = 12.3;

	homeGoal.xpos = 18;
	homeGoal.ypos = 126;
	homeGoal.zpos = 27;
	for (int i = 0; i<20;i++){
		outVals[i] = 420.69;
	}
  	//Task turrC(motorControl);
  	//Task varUP(updateInfoLoop);
	//Task sLoop(startLoop);
	//Task C2(controller2);
}

void disabled() {
	
	/*while(1){
		if (sidecar.get_digital_new_press(DIGITAL_L1)){
			chassis.teamColor = !chassis.teamColor;
		}
		if (sidecar.get_digital_new_press(DIGITAL_L2)){
			if (startPos == near){
				startPos = far;
			}else{
				startPos = near;
			}
		}
	}*/
}

void competition_initialize() {}

void autonomous(){
	chassis.teamColor = 0;
	startPos = far;
	autonomousReal();
}

void opcontrol() {
	//calibrateTurretDistances();
	chassis.driveTrain.running = false;
	PIDTunnerFly();
	// i want to go to world...
	while(1){/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//left normal speed and right normal speed (as in not using mechanum superpowers)
		chassis.driveTrain.leftSpd = -master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X);
		chassis.driveTrain.rightSpd = -master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X);

		//mechanum(magic) speed
		chassis.driveTrain.mechSpd = -master.get_analog(ANALOG_LEFT_X);

		//std::cout << "\nturenc:"<<double(turretEncoder.get_position())/2158.3333<<"chHeading:"<<-inertial.get_heading();
		liftConrol();
	
		if (usd::is_installed()){
			outPosSDCARD();
        	outValsSDCard();
		}

		devCheck();

		//have a 150 msec delay because of updating controller screen
		delay(20);

	}
}
