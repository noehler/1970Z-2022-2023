#include "main.h"
#include "Autons/autonSetup.h"
#include "Autons/winPointAuton.h"
#include "bezierCalculations.h"
#include "devFuncs.h"
#include "motorControl.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"
#include "sdLogging.h"
#include <iostream>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  setupScreen();
  motorControl_t mc;
  mc.setpistons();
  sensing.Init();

  Task odometry_Task(odometry_Wrapper, (void *)&sensing, "Odometry Task");
  Task gps_Task(GPS_Wrapper, (void *)&sensing, "GPS Task");
  sensing.set_status(37,18,270,100, 0);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  autonType = winPointBoth;
  color = 0;
  winPointAuton();

  if (autonType == winPointClose) { // close win Point auton
  } else if (autonType == winPointFar) { // far win Point auton
                                         
  } else if (autonType == winPointBoth) {//both roller win point
    winPointAuton();
  } else if (autonType == noAuton) {//noAuton
  } else {                          //skills Auton
    skillsAutonomous();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
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
void opcontrol() {
  motorControl_t motorControl;
  //motorControl.flyTune();

  Task drive_Task(drive_ControllerWrapper, (void *)&motorControl,
                  "My Driver Controller Task");
  Task turret_Intake_Task(turretIntake_ControllerWrapper, (void *)&motorControl,
                          "Intake and Turret Controller Task");
  Task fly_Task(fly_ControllerWrapper, (void *)&motorControl,
                "My Flywheel Speed Controller Task");
  Task SSOSTTT_Task(SSOSTTT_Wrapper, (void *)&sensing, "turret angle Task");
  Task vibrationTask(vibrateController,"My Vibrator Controller Task");
  
  while (1) {  
    if (master.get_digital_new_press(DIGITAL_A)){
      motorControl.discCountChoice = 2;
    }
    if (master.get_digital_new_press(DIGITAL_B)){
      motorControl.discCountChoice = 1;
    }

    sensing.SSOSTTT_bool = true;
    static bool autoAim = false;
    static int aimSpot = 0;
    if (master.get_digital_new_press(DIGITAL_UP)) {
      autoAim = !autoAim;
    }
    if (autoAim == false) {
      sensing.robot.turretLock = true;
      goalAngle = sensing.robot.angle + 180;
      sensing.goalSpeed = 200;
    } else {
      sensing.robot.turretLock = false;
    }

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
    if (color != 2){
      if (master.get_digital_new_press(DIGITAL_DOWN) &&master.get_digital(DIGITAL_LEFT) ) {
        aimSpot+=1;
        if (aimSpot ==3){
          aimSpot = 0;
        }
        if(aimSpot == 0){
          
          sensing.goal.xpos = 20;
          sensing.goal.ypos = 124;
          sensing.goal.zpos = 30;
        }
        else if (aimSpot == 1){
          sensing.goal.xpos = 124;
          sensing.goal.ypos = 20;
          sensing.goal.zpos = 30;
        }
        else{
          sensing.goal.xpos = 72;
          sensing.goal.ypos = 72;

        }
      }
    }
    else{
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
    
    
    delay(20);
  }
}
