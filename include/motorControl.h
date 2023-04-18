#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "Autons/autonSetup.h"
#include "api.h"
#include "bezierCalculations.h"
#include "devFuncs.h"
#include "output.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"


using namespace pros;

/*main motor control thread, handles:
 *communication to motors
 *PID controllers
 *simple macros that are reliant on motor information
 */
class motorControl_t {
private:
  // declaring motors and solenoids, defined at start of public
  Motor lfD;
  Motor lmD;
  Motor lbD;
  Motor rfD;
  Motor rmD;
  Motor rbD;
  Motor flyWheel1;
  Motor intakeMotor;
  ADIDigitalOut expansion;
  ADIDigitalOut blocker;
  ADIDigitalOut shoot3;
  ADIDigitalOut raise_magazine;

  // delay used to control most functions, tuned to optimize performance while
  // not overdrawing power or affecting calculations
  int optimalDelay = 10;

  // direct voltage contoller over rides, bool used to enable these is declared
  // publicly
  int drivePowerR;
  int drivePowerL;
  int intakePower;

  // All PID systems combined into one class
  class tunedSystems_t {
  public:
    PID_t driveFR, driveSS, turret, flyWheel, rotate;
  } PID;

  // class that handles individual values for moveTo functions
  // defined as a class because used several times seperately in three different
  // functions
  class moveToInfoInternal_t {
  public:
    double ets;       // variable used to store differnce in angle in degrees
    double dist = 0;  // change of position
    double distR = 0; // chagne of right postion
    double distL = 0; // change of left position
    double PIDSS = 0; // PID turning speed
    double PIDFW = 0; // PID moveforward speed
    double PIDSpeedL = 0; // PID leftside speed
    double PIDSpeedR = 0; // PID rightside speed
    double PIDFWFLAT = 0; // variable used for keeping move forward speed < 100
    double PIDSSFLAT = 0; // variable used to store the power to turn
  };

  // variables used to pass information from outside threads to movement
  // functions, declared in public
  class moveToInfoExternal_t {
  public:
    double moveToxpos, moveToypos, // goal position, maily used in moveTo file
        targetHeading, // goal final heading, maily used in rotateTo, but can be
                       // used in other functions
        speed_limit = 100, // max speed, 0 to 100 coorelates to 0 to 12000 mv
                           // applied to motor
        errtheta = 5; // max angle difference before robot is stopped to turn to
                      // correct angle
    int moveToforwardToggle =
            1, // controls whether driving forward ( +1) or reverse ( -1)
        Stop_type = 0;    //
    double tolerance = 5; // distance to goal point where will return a
                          // completed value and stop the robot
    bool resetMoveTo; // resets all variables in moveToInternals class, useful
                      // for tracking if completed movement
  };

  // conversion from inches per second to rpm needed at flywheel
  double angularVelocityCalc(int number) {
    if (lockSpeed == true) {
      return 650;
    } else {
      return sensing.goalSpeed * 2.215 + 63.5;
    }
    //triple shot equation = v*1.4 + 84.5
    //double shot equation = v*1.28 + 42
    //single shot equation = v*2.215 + 63.5
  }

  // turret controller called in turret intake thread, returns power to run
  // motor at in mv from -12000 to 12000
  double intakeControl(void) {
    //return variable
    static int baseSPD;


    if (master.get_digital(E_CONTROLLER_DIGITAL_R2) || intakeRunning == 1) { //intake
      baseSPD = 12000;
    } else if (master.get_digital(E_CONTROLLER_DIGITAL_R1) || intakeRunning == 2) { //outtake
      baseSPD = -12000;
    } else if (intakeRunning == 3) { //run at set speed
      if (intakeMotor.get_actual_velocity() < intakespdTarget && baseSPD < 12000) {// increase force because too slow
        baseSPD += 100;
      } else if (intakeMotor.get_actual_velocity() > intakespdTarget && //decrease force because too fast
                 baseSPD > -12000) {
        baseSPD -= 100;
      } else {  //maintain speed because just right
        baseSPD = baseSPD;
      }
    } else { //stop
      baseSPD = 0;
    }
    return baseSPD;
  }

public:
  bool updatedAD = false; //variable to account for difference in timings between threads, comparisons only rake place when all values are update
  double angdiff; // diffence in angle between turret and goal
  int lockSpeed = 1;
  double intakespdTarget = 0;
  int intakeRunning; // variable to control switch between types of intake contol. 0 for stop, 1 for intake, 2 for outtake, 3 for goal speed
  double diffFlyWheelW; // difference in goal speed of flywheel verse actual speed
  bool autoAim = false;
  double slope = 2.215;
  ADIDigitalOut raise_intake;

  // Constructor to assign values to the motors and PID values
  motorControl_t(void)
      : lfD(1, E_MOTOR_GEARSET_06, true), lmD(2, E_MOTOR_GEARSET_06, true), lbD(3, E_MOTOR_GEARSET_06, false),
        rfD(9, E_MOTOR_GEARSET_06, false), rmD(8, E_MOTOR_GEARSET_06, false), rbD(7, E_MOTOR_GEARSET_06, true),
        flyWheel1(6, E_MOTOR_GEARSET_06, true),
        intakeMotor(4, E_MOTOR_GEARSET_06, true), expansion({{22, 'D'}}), blocker({{22, 'F'}}),
        shoot3({{22, 'A'}}), raise_magazine({{22, 'B'}}), raise_intake({{22,'C'}}) {
    
    //setting PID values begining of motorcontrol class
    PID.driveFR.p = 6;
    PID.driveFR.i = 0.01;
    PID.driveFR.d = 10;

    PID.driveSS.p = 5;
    PID.driveSS.i = 0.065;
    PID.driveSS.d = 5;

    PID.rotate.p = 4;
    PID.rotate.i = 0.065;
    PID.rotate.d = 10;

    PID.turret.p = 3.2;
    PID.turret.i = .01;
    PID.turret.d = 30;

    PID.turret.p2 = 0.15;
    PID.turret.i2 = 0.00000002;
    PID.turret.d2 = 14;

    PID.flyWheel.p = 1.82587;
    PID.flyWheel.i = 0.0219322;
    PID.flyWheel.d = 0.732158;
    PID.flyWheel.p2 = 1.84413;
    PID.flyWheel.i2 = 0.0221515;
    PID.flyWheel.d2 = 1.05928;
  }

  //calibrate flywheel voltage
  void calibrateVoltage(void){
    while(1){
      double voltage = 1200000;
      while (fabs(voltage) > 12000){
        voltage = getNum("Millivotls: ");
      }
      flyWheel1.move_voltage(voltage);
      
      bool trackable = getNum("1 if steady: ");

      if (trackable == 1){
        int startTime = millis();
        while(millis() - startTime < 5000){
          logValue("time", millis(), 0);
          logValue("voltage", voltage, 1);
          logValue("rpm", flyWheel1.get_actual_velocity(), 2);
          delay(20);
        }
    
        logValue("empty", 0, 0);
        logValue("empty", 0, 1);
        logValue("empty", 0, 2);
        delay(200);
      }
    }
  }

  // class to set variables need to control moveTo function
  moveToInfoExternal_t move;

  //Function to rotate To a set angle
  double HeadingTarget = 0;
  void rotateTo(bool resetIntegs) {
    double powerOutput;
    angdiff = HeadingTarget - sensing.robot.angle;
    while(angdiff > 180){
      angdiff-=360;
    }
    while(angdiff < -180){
      angdiff+=360;
    }
    static double previousangdiff = angdiff;
    static double integral = 0;
    if (resetIntegs){
      previousangdiff = angdiff;
    }
    if (fabs(angdiff) > move.errtheta+3) {//voltage and acceleration controller
      //pre determined value, if too large will overshoot, if too small, will undershoot(better to undershoot and restart than overshoot and enter a cycle of overshooting)
      double acceleration = .15;

      //if motor is too hot the motor will not be able to accelerate as well
      double motordecreaseConstant = .9;
      if (lfD.get_temperature() > 45){
        acceleration*=motordecreaseConstant;
      }
      if (lmD.get_temperature() > 45){
        acceleration*=motordecreaseConstant;
      }
      if (lbD.get_temperature() > 45){
        acceleration*=motordecreaseConstant;
      }
      if (rfD.get_temperature() > 45){
        acceleration*=motordecreaseConstant;
      }
      if (rmD.get_temperature() > 45){
        acceleration*=motordecreaseConstant;
      }
      if (rbD.get_temperature() > 45){
        acceleration*=motordecreaseConstant;
      }

      //first term is amount of loops until target is reached, second term is amount of loops to slow down at current speed
      double basePWR = 9000;
      if (fabs(angdiff) / fabs(angdiff - previousangdiff) >
          fabs(angdiff - previousangdiff) / acceleration) { //accelerate
        if (angdiff < 0) {
          powerOutput = -basePWR;
        } else {
          powerOutput = basePWR;
        }
      } else {//deccelerate
        if (angdiff < 0) {
          powerOutput = basePWR;
        } else {
          powerOutput = -basePWR;
        }
      }

      //storing value for velocity calculation
      previousangdiff = angdiff;
      integral = 0;

    } else if (fabs(angdiff) > move.errtheta){//PID controller in small ranges
      powerOutput = (angdiff * PID.rotate.p + integral * PID.rotate.i + (angdiff - previousangdiff)*PID.rotate.d)* 12000 / 127;
      integral += angdiff;
      if (powerOutput >12000){
        powerOutput = 12000;
      }
      else if (powerOutput <-12000){
        powerOutput = -12000;
      }
    }
    else{
      powerOutput = 0;
      integral = 0;
    }
    leftSpd = -powerOutput;
    rightSpd = powerOutput;
  }

  //function to travel to specific point
  void moveTo(bool resetIntegs) {

    //setting up variables and setting intially to 0
    static double integralFW;
    static double integralSS;
    if (resetIntegs){
      integralFW = 0;
      integralSS = 0;
    }
    if (integralFW == NAN){
      integralFW = 0;
    }
    if (integralSS == NAN){
      integralSS = 0;
    }

    //Calculating proportionate and derivative for lateral controller
    double etx = move.moveToxpos - sensing.robot.xpos; // change of x
    double ety = move.moveToypos - sensing.robot.ypos; // change of y
    double FW = sqrt(pow(etx, 2) + pow(ety, 2));
    static double prevFW = FW;
    if (resetIntegs){
      prevFW = FW;
    }

    //Calculating proportionate and derivative for rotational controller
    move.targetHeading = atan2(ety,etx);
    double currentheading = sensing.robot.angle / 180 * M_PI;
    angdiff = move.targetHeading - currentheading;
    while( angdiff > M_PI){
      angdiff-= 2*M_PI;
    }
    while( angdiff < -M_PI){
      angdiff+= 2*M_PI;
    }
    static double prevSS = angdiff;
    if (resetIntegs){
      prevSS = angdiff;
    }
    
    double straightOutput = 0;
    double turnOutput = 0;
    if (fabs(angdiff) < move.errtheta/180*M_PI){
      
      if (FW > 2*move.tolerance) {//voltage and acceleration controller
        //pre determined value, if too large will overshoot, if too small, will undershoot(better to undershoot and restart than overshoot and enter a cycle of overshooting)
        double acceleration = .019;

        //if motor is too hot the motor will not be able to accelerate as well
        double motordecreaseConstant = .9;
        if (lfD.get_temperature() > 45){
          acceleration*=motordecreaseConstant;
        }
        if (lmD.get_temperature() > 45){
          acceleration*=motordecreaseConstant;
        }
        if (lbD.get_temperature() > 45){
          acceleration*=motordecreaseConstant;
        }
        if (rfD.get_temperature() > 45){
          acceleration*=motordecreaseConstant;
        }
        if (rmD.get_temperature() > 45){
          acceleration*=motordecreaseConstant;
        }
        if (rbD.get_temperature() > 45){
          acceleration*=motordecreaseConstant;
        }

        //first term is amount of loops until target is reached, second term is amount of loops to slow down at current speed
        double radius = -FW/(2*sin(angdiff));
        double ratio = fabs((radius - 4.5) / (radius + 4.5));

        double basePWR = 12000* ratio;

        if (fabs(FW) / fabs(FW - prevFW) >
            fabs(FW - prevFW) / acceleration) { //accelerate
          if (FW < 0) {
            straightOutput = -basePWR;
          } else {
            straightOutput = basePWR;
          }
        } else {//deccelerate
          if (FW < 0) {
            straightOutput = basePWR;
          } else {
            straightOutput = -basePWR;
          }
        }

        //storing value for velocity calculation
        integralFW = 0;
        integralSS = 0;
        
        if (radius > 0){
          leftSpd = straightOutput*pow(ratio,2);
          rightSpd = straightOutput;
        }
        else{
          leftSpd = straightOutput;
          rightSpd = straightOutput*pow(ratio,2);
        }
        

      } else if (fabs(FW) > move.tolerance){//PID controller in small ranges

        //PID controllers
        straightOutput = (FW * PID.driveFR.p + integralFW * PID.driveFR.i + (FW - prevFW)*PID.driveFR.d)* 12000 / 127;
        turnOutput = (angdiff * PID.driveSS.p + integralSS * PID.driveSS.i + (angdiff - prevSS)*PID.driveSS.d)* 12000 / 127;

        integralFW += FW;
        integralSS += angdiff;
        double max = fabs(straightOutput) + fabs(turnOutput);
        if (max >12000){
          double min = fabs(straightOutput) - fabs(turnOutput);
          double ratio = min/max;

          straightOutput = straightOutput / max*12000;
          turnOutput = turnOutput/max*12000;
        }
        leftSpd = straightOutput + turnOutput;
        rightSpd = straightOutput - turnOutput;
      }
      else{
        integralFW = 0;
        integralSS = 0;
        
        leftSpd = 0;
        rightSpd = 0;

        move.resetMoveTo = true;
      }

    }
    else{ //if angle is too far off only use rotation
      HeadingTarget = move.targetHeading*180/M_PI;
      rotateTo(0);
    }

    //storing previous values for velocity calc
    prevFW = FW;
    prevSS = angdiff;
  }

  //set pistions to correct starting position to ensure that there is no early expansion or shooting at start of match
  void setpistons(void) {
    shoot3.set_value(false);
    raise_intake.set_value(false);
    raise_magazine.set_value(false);
    expansion.set_value(false);
  }

  //function to better follow paths generated by Bezier curves
  //NOT FUNCTIONAL AT THE MOMENT
  void tailGater(bez_Return_t temp) {
    //setting up intial variables that do not change
    static bool finished = false;
    static int current = 0;
    static double errorCurveInteg = 0;
    static double radiusScalar = 0;
    static double radiusDifference = 9.45 / 2;

    // setting starting to nan so they get set to 0 by if statements
    static double integralSS = NAN;
    static double integralFW = NAN;
    static double prevET = NAN;
    static double prevETS = NAN;
    while (finished == false) {
      while (1) {
        //calculating distance to next point
        double et =
            sqrt(pow(sensing.robot.xpos - temp.returnPoints[current][0], 2) +
                 pow(sensing.robot.ypos - temp.returnPoints[current][1], 2));

        if (et < 3 ) { ///////////////land mine
          current++;
          if (current == temp.length + 1) {
            finished = true;
          }
          break; // path finished exit
        }

        delay(10);

        // calculating dist left
        for (int i = current; i < temp.length - 3; i += 3) {
          et += sqrt(
              pow(temp.returnPoints[i + 3][0] - temp.returnPoints[i][0], 2) +
              pow(temp.returnPoints[i + 3][1] - temp.returnPoints[i][1], 2));
        }

        // calculating dist between
        // code to calculate distance and determine if negative or positive
        double ets;
        if (current != 0) {
          double m = (temp.returnPoints[current][1] -
                      temp.returnPoints[current - 1][1]) /
                     (temp.returnPoints[current][0] -
                      temp.returnPoints[current - 1][0]);
          double lineAngle = atan(m);
          if ((temp.returnPoints[current][0] -
               temp.returnPoints[current - 1][0]) < 0) {
            lineAngle = lineAngle + M_PI;
          } else {
            lineAngle = lineAngle + 2 * M_PI;
          }

          double robotToPoint = atan((temp.returnPoints[current][1] -
                                      temp.returnPoints[current - 1][1]) /
                                     (temp.returnPoints[current][0] -
                                      temp.returnPoints[current - 1][0]));
          if ((temp.returnPoints[current][0] -
               temp.returnPoints[current - 1][0]) < 0) {
            robotToPoint = robotToPoint + M_PI;
          } else {
            robotToPoint = robotToPoint + 2 * M_PI;
          }

          double angleDiff = lineAngle - robotToPoint;
          int multiplier = fabs(angleDiff) / angleDiff;

          ets = fabs(-m * sensing.robot.xpos + sensing.robot.ypos +
                     m * temp.returnPoints[current][0] -
                     temp.returnPoints[current][1]) /
                sqrt(pow(temp.returnPoints[current][1], 2) + 1);
        } else {
          ets = 0;
        }

        // checking if NANs
        if (isnanf(prevET)) {
          prevET = et;
        }
        if (isnanf(prevETS)) {
          prevETS = ets;
        }
        if (isnanf(integralFW)) {
          integralFW = 0;
        }
        if (isnanf(integralSS)) {
          integralSS = 0;
        }

        // calculating forward speed and updating integrals/derivatives
        double FWVal = et * PID.driveFR.p + integralFW * PID.driveFR.i +
                       (et - prevET) * PID.driveFR.d;
        integralFW += et;
        prevET = et;

        double radIn;
        double radOut;
        radIn = (temp.radius[current] * radiusScalar - radiusDifference);
        radOut = (temp.radius[current] * radiusScalar + radiusDifference);

        if (FWVal * radOut / radIn > 127) {
          FWVal = FWVal * radIn / radOut;
        }

        // calculating side speed and updating integrals/derivatives
        double SSVal = ets * PID.driveSS.p + integralSS * PID.driveSS.i +
                       (ets - prevETS) * PID.driveSS.d;
        integralSS += ets;
        prevETS = ets;

        if (fabs(FWVal) + fabs(SSVal) > 127) {
          FWVal = (fabs(FWVal) - fabs(SSVal)) * FWVal / fabs(FWVal);
        }

        double PIDSpeedL = FWVal - SSVal;
        double PIDSpeedR = FWVal + SSVal;

        if (ets > 0) {
          PIDSpeedL = PIDSpeedL * fabs(radIn / radOut);
          PIDSpeedR = PIDSpeedR * fabs(radOut / radIn);
        } else {
          PIDSpeedR = PIDSpeedR * fabs(radIn / radOut);
          PIDSpeedL = PIDSpeedL * fabs(radOut / radIn);
        }

        lfD.move(PIDSpeedL);
        lbD.move(PIDSpeedL);
        rfD.move(PIDSpeedR);
        rbD.move(PIDSpeedR);
        delay(20);
      }
    }

    lfD.brake();
    lbD.brake();
    rfD.brake();
    rbD.brake();
  }

  //Auton drive controller and variables associated with it
  int driveType = 0;// 0 = moveTo, 1 = rotateTo, 2 = driveVoltage
  double leftSpd = 0; //variables to dodo direct voltage control for elsewhere in the class
  double rightSpd = 0;//variables to dodo direct voltage control for elsewhere in the class
  void autonDriveController(void) {
    //check to ensure that thread stops when auton stops to ensure no interference with driver control
    while (!competition::is_disabled() && competition::is_autonomous()) {

      //setting drive type to impossible number to ensure that variables are reset at start of function
      static int prevdriveType = 1000;
      bool resetIntegs = false;
      if (driveType != prevdriveType) {
        resetIntegs = true;
      }
      //resets all integrals at when changing from one drive type to another

      switch (driveType) {
      case 0: // moveTo
        moveTo(resetIntegs);
        break;
      case 1: // rotateTo
        rotateTo(resetIntegs);
        break;
      case 2: // driveVoltage
        break;
      case 3://aim
        HeadingTarget = goalAngle;
        rotateTo(resetIntegs);
        break;
      }
      prevdriveType = driveType;

      //ensuring that inputs do not exceed the capabilities of the motor
      if (leftSpd > 12000) {
        leftSpd = 12000;
      }
      if (rightSpd > 12000) {
        rightSpd = 12000;
      }

      if (leftSpd < -12000) {
        leftSpd = -12000;
      }
      if (rightSpd < -12000) {
        rightSpd = -12000;
      }

      //changing how the motor controls deceleration and stopping
      if (move.Stop_type == 1) {
        lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        lmD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rmD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
      } else if (move.Stop_type == 0) {
        lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        lmD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rmD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
      } else {
        lfD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        lmD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        lbD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        rfD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        rmD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        rbD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
      }

      //driving or stopping the motor
      if (drivePowerL == 0 && drivePowerR == 0) {
        lfD.brake();
        lmD.brake();
        lbD.brake();
        rfD.brake();
        rmD.brake();
        rbD.brake();
      } else {
        lfD.move_voltage(leftSpd);
        lmD.move_voltage(leftSpd);
        lbD.move_voltage(leftSpd);
        rfD.move_voltage(rightSpd);
        rmD.move_voltage(rightSpd);
        rbD.move_voltage(rightSpd);
      }

      angdiff = goalAngle - sensing.robot.angle;
      updatedAD = true;
      //outpuutting for match review
      logValue("x", sensing.robot.xpos, 0);
      logValue("y", sensing.robot.ypos, 1);
      logValue("ODOx", sensing.robot.odoxpos, 2);
      logValue("ODOy", sensing.robot.odoypos, 3);
      logValue("moveToX", move.moveToxpos, 4);
      logValue("moveToy", move.moveToypos, 5);
      logValue("heading", sensing.robot.angle, 6);
      logValue("turr ang", sensing.robot.turAng, 7);
      logValue("angDiff", angdiff, 8);
      logValue("batt pct", battery::get_capacity(), 9);
      logValue("lfdTemp", lfD.get_temperature(), 10);
      logValue("lmdTemp", lmD.get_temperature(), 11);
      logValue("lbdTemp", lbD.get_temperature(), 12);
      logValue("rfdTemp", rfD.get_temperature(), 13);
      logValue("rmdTemp", rmD.get_temperature(), 14);
      logValue("rbdTemp", rbD.get_temperature(), 15);
      logValue("intakeTemp", intakeMotor.get_temperature(), 16);
      logValue("fW1Temp", flyWheel1.get_temperature(), 17);
      logValue("fw1SPD", flyWheel1.get_actual_velocity(), 18);
      logValue("magCount", sensing.robot.magFullness, 19);
      logValue("goalAngle", goalAngle, 20);
      logValue("goalSPD", sensing.goalSpeed, 21);
      logValue("goalSPD", angularVelocityCalc(sensing.goalSpeed), 22);
      //logging color from rollerGood
      logValue("rollerGoodFRONT", sensing.rollerIsGood(1),26);
      logValue("rollerGoodBACK", sensing.rollerIsGood(-1),27);
      logValue("time", millis(), 28);

      delay(optimalDelay);
    }
    lfD.brake();
    lmD.brake();
    lbD.brake();
    rfD.brake();
    rmD.brake();
    rbD.brake();
  }

  // Operator control drive controller
  void driveController() {
    while (!competition::is_disabled()) {
      //setting variables and converting them to millivolts
      double fwdSPD = double(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * 12000 / 127;
      double turnSPD;
      static bool prevAutoAim;
      if (autoAim && abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 3){
        HeadingTarget = goalAngle;
        move.errtheta = 1;
        rotateTo(prevAutoAim != autoAim);
        turnSPD = leftSpd;
      }
      else{
        turnSPD = double(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) * 12000 / 127;
      }
      prevAutoAim = autoAim;

      leftSpd = fwdSPD + turnSPD;
      rightSpd = fwdSPD - turnSPD;

      if (leftSpd == 0 && rightSpd == 0) {
        lfD.brake();
        lmD.brake();
        lbD.brake();
        rfD.brake();
        rmD.brake();
        rbD.brake();
      } else {
        lfD.move_voltage(leftSpd);
        lmD.move_voltage(leftSpd);
        lbD.move_voltage(leftSpd);
        rfD.move_voltage(rightSpd);
        rmD.move_voltage(rightSpd);
        rbD.move_voltage(rightSpd);
      }

      angdiff = goalAngle - sensing.robot.angle;
      updatedAD = true;

      
      //outpuutting for match review
      logValue("x", sensing.robot.xpos, 0);
      logValue("y", sensing.robot.ypos, 1);
      logValue("ODOx", sensing.robot.odoxpos, 2);
      logValue("ODOy", sensing.robot.odoypos, 3);
      logValue("moveToX", move.moveToxpos, 4);
      logValue("moveToy", move.moveToypos, 5);
      logValue("heading", sensing.robot.angle, 6);
      logValue("turr ang", sensing.robot.turAng, 7);
      logValue("angDiff", angdiff, 8);
      logValue("batt pct", battery::get_capacity(), 9);
      logValue("lfdTemp", lfD.get_temperature(), 10);
      logValue("lmdTemp", lmD.get_temperature(), 11);
      logValue("lbdTemp", lbD.get_temperature(), 12);
      logValue("rfdTemp", rfD.get_temperature(), 13);
      logValue("rmdTemp", rmD.get_temperature(), 14);
      logValue("rbdTemp", rbD.get_temperature(), 15);
      logValue("intakeTemp", intakeMotor.get_temperature(), 16);
      logValue("fW1Temp", flyWheel1.get_temperature(), 17);
      logValue("fw1SPD", flyWheel1.get_actual_velocity(), 18);
      logValue("magCount", sensing.robot.magFullness, 19);
      logValue("goalAngle", goalAngle, 20);
      logValue("goalSPD", sensing.goalSpeed, 21);
      logValue("goalSPD", angularVelocityCalc(sensing.goalSpeed), 22);
      //logging color from rollerGood
      logValue("rollerGoodFRONT", sensing.rollerIsGood(1),26);
      logValue("rollerGoodBACK", sensing.rollerIsGood(-1),27);
      logValue("time", millis(), 28);

      delay(optimalDelay);
    }
  }

  // voltage controller for flywheel motors
  double flyAngularVelocity;
  void flyController() {
    // speed needed to accelerate
    double bottomLimit = 25;
    // speed needed to decelerate
    double topLimit = 30;

    // setting up PID controller values
    double angularVelocityDifferenceIntegral1 = 0;
    double prevAngularVelocityDifference1 = 0;
    
    while (!competition::is_disabled()) {
      flyAngularVelocity = flyWheel1.get_actual_velocity();
      if (flyWheel1.get_temperature() < 45) {
        double flyWVolt1;
        double goalAngularVelocity = angularVelocityCalc(sensing.goalSpeed);
        double holdPower1 = (17.45 * goalAngularVelocity) + 994;

        // positive values means acceleration is needed and negative means speed
        // is too high
        diffFlyWheelW =
            goalAngularVelocity - flyWheel1.get_actual_velocity();

        // checking nanf
        if (angularVelocityDifferenceIntegral1 == NAN) {
          angularVelocityDifferenceIntegral1 = 0;
        }

        // flywheel1 controller
        if (diffFlyWheelW > bottomLimit) { // not fast enough
          angularVelocityDifferenceIntegral1 = 0;
          flyWVolt1 = 12000;
        } else if (diffFlyWheelW < -topLimit) { // too fast
          angularVelocityDifferenceIntegral1 = 0;
          flyWVolt1 = -12000;
        } else {
          double prop = diffFlyWheelW * PID.flyWheel.p * 0;
          double integ =
              angularVelocityDifferenceIntegral1 * PID.flyWheel.i * 0;
          if (!(integ > 60)) {
            angularVelocityDifferenceIntegral1 += diffFlyWheelW;
          }
          double deriv =
              (prevAngularVelocityDifference1 - diffFlyWheelW) *
              PID.flyWheel.d * 0;
          double PIDVAL = (prop + integ + deriv) * 12000 / 127 * 0;
          angularVelocityDifferenceIntegral1 += diffFlyWheelW;
          flyWVolt1 = holdPower1 + PIDVAL;
        }

        prevAngularVelocityDifference1 = diffFlyWheelW;

        flyWheel1.move_voltage(flyWVolt1);
      } else {
        static double IPIDang = 0;
        static double IPIDang2 = 0;

        //calculating how diffence in speed as input for PID and as tool to know when to shoot
        double flyWVolt;
        double flyWheelW = flyWheel1.get_actual_velocity();
        diffFlyWheelW =
            angularVelocityCalc(sensing.robot.magFullness) - flyWheelW;
        static double prevFWdiffSPD =
            angularVelocityCalc(sensing.robot.magFullness);

        //updating Integral and derivative, calculating PID
        IPIDang += diffFlyWheelW;
        double prop = PID.flyWheel.p * diffFlyWheelW;
        double integ = IPIDang * PID.flyWheel.i;
        double integ2 = IPIDang2 * PID.flyWheel.i2;
        double deriv = PID.flyWheel.d * (diffFlyWheelW - prevFWdiffSPD);
        prevFWdiffSPD = diffFlyWheelW;


        //converting to millivolts and ensuring that it does not exceed the capabilities of the motors
        flyWVolt = 12000.0 / 127 * (prop + integ + deriv);

        if (flyWVolt > 12000) {
          flyWVolt = 12000;
        }
        if (flyWVolt < -12000) {
          flyWVolt = -12000;
        }
        if (fabs(diffFlyWheelW) < 1 && flyWVolt == 0) {
          IPIDang = 0;
        }

        //checking for not a number returns, happens often when motor is disconnected
        if (isnanf(flyWVolt)) {
          flyWVolt = 0;
          prevFWdiffSPD = angularVelocityCalc(sensing.robot.magFullness);
          IPIDang = 0;
        }

        flyWheel1.move_voltage(flyWVolt);
      }

      delay(optimalDelay);
    }
  }

  // power controller for that run intake and turret
  bool runTurretIntake = true;//variable to properly stop the thread
  void intakeController() {
    runTurretIntake = true;
    while (!competition::is_disabled() && runTurretIntake == true) {

      if (!competition::is_autonomous()) {//control pistons in driver control
        if (master.get_digital(E_CONTROLLER_DIGITAL_UP) &&//expand
            master.get_digital(E_CONTROLLER_DIGITAL_LEFT)) {
          expansion.set_value(true);
        } 
        if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN) &&//block
            master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
          blocker.set_value(true);
        } 

        raise_intake.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));//operates piston to raise intake
        shoot3.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
        raise_magazine.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP));//operates piston lift magazine pressure
      }

      double intakeSpd = intakeControl();

      intakeMotor.move_voltage(intakeSpd);

      delay(optimalDelay);
    }
    intakeMotor.brake();
  }

  // actuates the pistons during autonomous
  void raiseAScore(int number) {
    if (number == 3){
      shoot3.set_value(true);
      delay(1000);
      shoot3.set_value(false);
    }
    else{
      intakeRunning = 2;
      delay(500);
      intakeRunning = 0;
    }
  }

  // moves to roller and spins it to correct orientation
  //fwd == 1 || -1
  void driveToRoller(double time, int sensor, bool reverseout = true) { // changing to let move to take
                                               // over the initial moveto
    bool finished = false;
    driveType = 2;//manual voltage control

    int startTime = millis();
    while (!sensing.underRoller(1) && !sensing.underRoller(2) && //drive to roller until under roller
           millis() - startTime < time) {
      leftSpd = -5000;
      rightSpd = -5000;
      delay(20);
    }
    //run with constant power to ensure contact with roller
    leftSpd = -4000;
    rightSpd = -4000;

    intakeRunning = 1;//speed control for intake
    sensing.rollerIsGood(sensor, 1);
    while (finished == false && millis() - startTime < time) {//wait until roller is flipped
      if (sensing.rollerIsGood(sensor)) {
        finished = true;
        break;
      }
      delay(20);
    }

    if (reverseout) {
      leftSpd = 8000;
      rightSpd = 8000;
      intakeRunning = 1;
      delay(150);
    }
    leftSpd = 0;
    rightSpd = 0;
    intakeRunning = 0;
  }

  // expansion
  void explode(void) {
    expansion.set_value(true);
    driveType = 2;
    rightSpd = 0;
    leftSpd = 0;
  }
};

//wrappers that exist to start threads inside a class
extern void drive_ControllerWrapper(void *mControl);
extern void intake_ControllerWrapper(void *mControl);
extern void fly_ControllerWrapper(void *mControl);

#endif