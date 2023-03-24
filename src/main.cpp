#include "main.h"
#include "Autons/autonSetup.h"
#include "devFuncs.h"
#include "output.h"
#include "pros/rtos.h"
#include "robotConfig.h"

//first section of code that runs in the robot
void initialize() {
	//function that starts up GUI and controls it for the rest of the match
	setupScreen();

	//setting initial pistion positions and calibrating inertial sensors
	motorControl_t mc;
	mc.setpistons();
	sensing.Init();

	//positional tracking threads starting
	Task odometry_Task(odometry_Wrapper, (void *)&sensing, "Odometry Task");
	Task gps_Task(GPS_Wrapper, (void *)&sensing, "GPS Task");

	//logging and interface threads starting up
	Task AutonSelector_Task(AutonSelector, "Auton Selector Task");
	Task outPutting_task(outValsSDCard, "Outputting Task");

	//default values set in case of entering into operatorControl without going through autonomous first
	sensing.set_status(37,18,270,100, 0);
}

//section of code that runs when robot is not in autonomous or driver
void disabled() {}

//seperate intialization procedure that runs only when plugged into field controller
void competition_initialize() {}

//section of code that runs at start of autonomous, threads are started locally in each seperate function
void autonomous() {
	logMessage("start of autonomous");
	switch(autonType){
		case winPointClose: // close win Point auton
    		closeWinPoint();
			break;
		case winPointFar://    far win Point auton
    		farWinPoint();
			break;
		case winPointBoth://   both roller win point
    		winPointAuton();
			break;
		case skillsAuton://	   skills auton
    		skillsAutonomous();
			break;
		case noAuton://   	   noAuton
			break;
		
	}
	logMessage("end of autonomous");
	while(1){
		delay(20);
	}
}


//secotion of code used to alert driver when all conditions right to shoot
bool vibrate = false;
void vibrateController(void){
  while(1){
    while(!vibrate){
      delay(20);
    }
    vibrate = false;
    master.rumble(".");
    delay(50);
  }
}

//Main driver control thread that runs during matches, motor control threads started locally here
void opcontrol() {
	logMessage("start of operator control");
	//starting up tasks neccessary for driving
	motorControl_t motorControl;

	Task drive_Task(drive_ControllerWrapper, (void *)&motorControl,
					"My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void *)&motorControl,
							"Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void *)&motorControl,
					"My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void *)&sensing, "turret angle Task");
	Task vibrationTask(vibrateController,"My Vibrator Controller Task");
  
	while (1) {  
		if (master.get_digital_new_press(DIGITAL_L2)){
			logMessage("shoot 1");
		}
		if (master.get_digital_new_press(DIGITAL_L1)){
			logMessage("shoot 3");
		}

		//adjusts goal speed for single vs double shot
		if (master.get_digital_new_press(DIGITAL_A)){
			motorControl.discCountChoice = 2;
		}
		if (master.get_digital_new_press(DIGITAL_B)){
			motorControl.discCountChoice = 1;
		}

		//adjusts goal angle to shoot at goal or able to intake
		sensing.SSOSTTT_bool = true;
		static bool autoAim = false;
		static int aimSpot = 0;
		if (master.get_digital_new_press(DIGITAL_UP)) {
			autoAim = !autoAim;
		}
		if (autoAim == false) {
			sensing.robot.turretLock = true;
			goalAngle = sensing.robot.angle + 180;
		} else {
			sensing.robot.turretLock = false;
		}

		//code to alert driver when angle and speed of turret are correct
		static bool turretGood = false;
		if(fabs(motorControl.diffFlyWheelW) <35 && motorControl.angdiff < 3 && sensing.robot.turretLock == false){
			if (turretGood == false){
				vibrate = true;
			}
			turretGood = true;
		}
		else{
			turretGood = false;
		}

		//code to change goal shooting position
		if (color != 2){
			if(master.get_digital(DIGITAL_LEFT)){
				//our goal
				sensing.goal.xpos = 20;
				sensing.goal.ypos = 124;
				sensing.goal.zpos = 30;
			}
			else if (master.get_digital(DIGITAL_RIGHT)){
				//middle of field
				//for expansion
				sensing.goal.xpos = 72;
				sensing.goal.ypos = 72;
			}
		}
		else{
			//automatic calculation for skills
			if (sensing.robot.xpos > sensing.robot.ypos){
				sensing.goal.xpos = 124;
				sensing.goal.ypos = 20;
				sensing.goal.zpos = 30;
			}
			else{
				sensing.goal.xpos = 20;
				sensing.goal.ypos = 124;
				sensing.goal.zpos = 30;
			}
		}
		
		//GPS Calibration
		static int loop = 0;
		static double xGPSintegral = 0;
		static double yGPSintegral = 0;
		static bool calculatePos = false;
		if (master.get_digital(DIGITAL_Y)){
			xGPSintegral +=sensing.robot.GPSxpos;
			yGPSintegral +=sensing.robot.GPSypos;
			loop++;
			calculatePos = true;
		}
		else{
			if (calculatePos && loop > 50){
				sensing.robot.xpos = xGPSintegral/loop;
				sensing.robot.ypos = yGPSintegral/loop;

				xGPSintegral = 0;
				yGPSintegral = 0;
				loop = 0;
				calculatePos = 0;
			}
			else{
				xGPSintegral = 0;
				yGPSintegral = 0;
				calculatePos = 0;
				loop = 0;
			}
		}
		
		delay(20);
	}
}
