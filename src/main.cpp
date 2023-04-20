#include "main.h"
#include "Autons/autonSetup.h"
#include "devFuncs.h"
#include "output.h"
#include "pros/misc.hpp"
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

	//logging and interface threads starting up
	Task AutonSelector_Task(AutonSelector, "Auton Selector Task");
	Task outPutting_task(outValsSDCard, "Outputting Task");

	//default values set in case of entering into operatorControl without going through autonomous first
	sensing.set_status(37,11.5,90,100, 1);
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
		delay(200);
	}
}


//secotion of code used to alert driver when all conditions right to shoot
bool vibrate = false;
void vibrateController(void){
  while(!competition::is_disabled()){
    while(vibrate){
		master.rumble(".");
		delay(400);
	}
	delay(200);
  }
}

//Main driver control thread that runs during matches, motor control threads started locally here
void opcontrol() {
	logMessage("start of operator control");

	//starting up tasks neccessary for driving
	motorControl_t motorControl;
	Task drive_Task(drive_ControllerWrapper, (void *)&motorControl,
					"My Driver Controller Task");
	Task turret_Intake_Task(intake_ControllerWrapper, (void *)&motorControl,
							"Intake Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void *)&motorControl,
					"My Flywheel Speed Controller Task");
	Task speedAngleCalc_Task(speedAngleCalc_Wrapper, (void *)&sensing, "turret angle Task");
	Task vibrationTask(vibrateController,"My Vibrator Controller Task");
	motorControl.autoAim = false;
  
	while (1) {  
		std::cout << "\n\nlooping \n\n";
		if (master.get_digital_new_press(DIGITAL_R1)){
			logMessage("reverse intake");
			sensing.shooting(motorControl.flyAngularVelocity);
		}
		if (master.get_digital_new_press(DIGITAL_L1)){
			logMessage("shoot 3");
		}

		//aim and speed control
		static bool aPressed = false;
		static bool bPressed = false;
		bool bRun = false;
		bool aRun = false;
		if (master.get_digital(DIGITAL_B)){
			if (bPressed == false){
				bRun = true;
			}
			bPressed = true;
		}
		else{
			bPressed = false;
		}
		if (master.get_digital(DIGITAL_A)){
			if (aPressed == false){
				aRun = true;
			}
			aPressed = true;
		}
		else{
			aPressed = false;
		}

		if (aRun == true){
			if (motorControl.lockSpeed == 1){
				motorControl.autoAim = true;
				motorControl.lockSpeed = 0;
			}
			else if (motorControl.autoAim == 0){
				motorControl.autoAim = true;
				motorControl.lockSpeed = 0;
			}
			else{
				motorControl.autoAim = false;
				motorControl.lockSpeed = 1;
			}
		}

		if (bRun == true){
			if (motorControl.lockSpeed == 0){
				motorControl.autoAim = true;
				motorControl.lockSpeed = 1;
			}
			else if (motorControl.autoAim == 0){
				motorControl.autoAim = true;
				motorControl.lockSpeed = 1;
			}
			else{
				motorControl.autoAim = false;
				motorControl.lockSpeed = 1;
			}
		}
 		
		//code to alert driver when angle and speed of turret are correct
		static bool turretGood = false;
		if(fabs(motorControl.diffFlyWheelW) <35 && motorControl.angdiff < 3){
			vibrate = true;
		}
		else{
			vibrate = false;
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
		std::cout << "loop done\n";
		delay(25);
	}
}
