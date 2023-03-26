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
  Motor lbD;
  Motor rfD;
  Motor rbD;
  Motor flyWheel1;
  Motor flyWheel2;
  Motor turretMotor;
  Motor intakeMotor;
  ADIDigitalOut boomShackalacka;
  ADIDigitalOut shoot3;
  ADIDigitalOut shoot1;
  ADIDigitalOut ejectPiston;

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
    PID_t driveFR, driveSS, turret, flyWheel;
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
    if (number == 3 && discCountChoice == 2) {
      return sensing.goalSpeed * 1.4 + 55.0*discMults[2][0];
    } else if (number == 2 && discCountChoice == 2) {
      return sensing.goalSpeed * 1.28 + 42*discMults[1][0];
    } else {
      return sensing.goalSpeed * 1.12 + 35*discMults[0][0];
    }
  }

  // turret controller called in turret intake thread, returns power to run
  // motor at in mv from -12000 to 12000
  double turrControl(void) {
    // uses a double PID, one to account for the angle difference and another to
    // account for the velocity of the robot angular difference is denoted as
    // Position velocity PID is denoted with velocity

    /*  setting up variables needed for PID controller  */
    // variables that output the final power to run the motors at
    static double PIDPosition = 0;
    static double PIDVelocity = 0;

    // variables that store the integrals for the PID controllers
    static double IPIDposition = 0;
    static double IPIDvelocity = 0;

    // delta T calculation used in the Velocity PID
    static double T = 0;
    static double previousT = 0;
    T = float(millis()) / 1000 - previousT;
    previousT += T;

    //different multipliers to account for movement and rotation of the robot
    static double PIDscalar = 5;
    static double gyroScalar = 0.2;
    static double chassisScalar = 0.35;
    static double turPredicScalar = .7;

    //modded calculation for the angle difference between the goal angle and actual angle
    angdiff = goalAngle - sensing.robot.turAng;
    if (angdiff > 180) {
      angdiff -= 360;
    } else if (angdiff < -180) {
      angdiff += 360;
    }

    //calculation to determine if the goal position will be between +- 180 degrees relative to the robot
    //used to make sure that turret does not over rotate in one direction and lock out due to wires running through the middle
    double robotAngleDiff = goalAngle - sensing.robot.angle;
    double turretAngle = double(sensing.turretEncoder.get_position()) / 100 +
                         180 - highTurretInitAng * 360;
    if (turretAngle + angdiff > 405) {
      angdiff -= 360;
    }
    if (turretAngle + angdiff < -45) {
      angdiff += 360;
    }
    //variable used in shooting macros to account for possible mis alignment of function timing. 
    //If not in place, macro could update the goal position and then do a check of position before value actually updated
    updatedAD = true;

    //variables to calculate the derivative
    static double previousveldiff = 0;
    static double previousangdiff = 0;

    //stop controller and reset all integrals if unable to move turret
    if (!competition::is_disabled()) {
      if (fabs(angdiff) > 10) {//voltage and acceleration controller
        //pre determined value, if too large will overshoot, if too small, will undershoot(better to undershoot and restart than overshoot and enter a cycle of overshooting)
        double acceleration = .5;

        //if motor is too hot the motor will not be able to accelerate as well
        if (turretMotor.get_temperature() > 45){
          acceleration*=.3;
        }

        //first term is amount of loops until target is reached, second term is amount of loops to slow down at current speed
        if (fabs(angdiff) / fabs(angdiff - previousangdiff) >
            fabs(angdiff - previousangdiff) / acceleration) { //accelerate
          if (angdiff < 0) {
            PIDPosition = -127;
          } else {
            PIDPosition = 127;
          }
        } else {//deccelerate
          if (angdiff < 0) {
            PIDPosition = 127;
          } else {
            PIDPosition = -127;
          }
        }

        //reseting integrals to get a new PID when within range to use PID
        previousangdiff = angdiff;
        IPIDposition = 0;
        IPIDvelocity = 0;
        return PIDPosition;

      } else if (fabs(angdiff) > 1){//PID controller in case of overheat
        //Anglular PID
        IPIDposition += angdiff;
        PIDPosition = (PID.turret.p * angdiff + PID.turret.i * IPIDposition +
                       PID.turret.d * (angdiff - previousangdiff));


        //velocity PID
        if (sensing.robot.turretLock && fabs(angdiff) < 5) {
          gyroScalar = 0;
          chassisScalar = 0;
          turPredicScalar = 0;
        } else {
          gyroScalar = 0.2;
          chassisScalar = 0.4;
          turPredicScalar = 3;
        }

        double veldiff = gyroScalar * T * (sensing.robot.angAccel) -
                         sensing.robot.velW * chassisScalar +
                         turPredicScalar * sensing.robot.turvelocity +
                         PIDPosition * PIDscalar;

        IPIDvelocity += veldiff;
        PIDVelocity = (1 * veldiff + 0.01 * IPIDvelocity +
                       0.1 * (veldiff - previousveldiff));
        previousveldiff = veldiff;

        //clearing variables if within range
        if (fabs(angdiff) < 1 && PIDPosition == 0 && fabs(veldiff) < 1) {
          IPIDvelocity = 0;
        }
        if (fabs(veldiff) < 0.1) {
          IPIDvelocity = 0;
        }
        if (fabs(angdiff) < 1 && fabs(angdiff - previousangdiff) < 10) {
          return 0;
        }
        previousangdiff = angdiff;
      }
      else{//hold motor position
        IPIDvelocity = 0;
        IPIDposition = 0;
        return 0;
      }
    } else {//resetting intergrals and holding if angle diff < 1
      PIDVelocity = 0;
      IPIDvelocity = 0;
      IPIDposition = 0;
    }
    
    //removing any power if overall power output is too low
    if (fabs(PIDVelocity) < 0.01) {
      return 0;
    }

    //reseting variables if corrupted by nan, can be caused by sensor error
    if (isnanf(PIDVelocity)) {
      PIDVelocity = 0;
      IPIDvelocity = 0;
      IPIDposition = 0;
    }

    return PIDVelocity;
  }

  // turret controller called in turret intake thread, returns power to run
  // motor at in mv from -12000 to 12000
  double intakeControl(void) {
    //return variable
    static int baseSPD;

    //setting up variables for jam detection
    static int jamTime = -9000;//negative number so that jam can be cleared immediately
    static int startTime = millis();
    static bool switched = false;


    if (master.get_digital(E_CONTROLLER_DIGITAL_R2) || intakeRunning == 1) { //intake
      if (switched == false) {
        startTime = millis();
        switched = true;
      }
      if (millis() - jamTime > 500) {//normal operation
        baseSPD = 12000;
      } else { //reverse to clear jam
        baseSPD = -12000;
        startTime = millis();
      }
      if (millis() - startTime > 250 && millis() - jamTime > 500 && //start jam movement if motor is too slow and long enough after motor started moving
          fabs(intakeMotor.get_actual_velocity()) < 5) {
        jamTime = millis();
      }

    } else if (master.get_digital(E_CONTROLLER_DIGITAL_R1) || //outtake
               intakeRunning == 2) {
      baseSPD = -12000;
      switched = false;

    } else if (intakeRunning == 3) { //run at set speed
      switched = false;
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
      switched = false;
    }
    return baseSPD;
  }

public:
  bool updatedAD = false; //variable to account for difference in timings between threads, comparisons only rake place when all values are update
  double angdiff; // diffence in angle between turret and goal
  int discCountChoice = 2; // 1 for single shot, 2 for automatic decision
  double intakespdTarget = 0;
  int intakeRunning; // variable to control switch between types of intake contol. 0 for stop, 1 for intake, 2 for outtake, 3 for goal speed
  double diffFlyWheelW; // difference in goal speed of flywheel verse actual speed
  double diffFlyWheelW2; // difference in goal speed of flywheel verse actual speed

  // Constructor to assign values to the motors and PID values
  motorControl_t(void)
      : lfD(5, E_MOTOR_GEARSET_06, true), lbD(4, E_MOTOR_GEARSET_06, false),
        rfD(2, E_MOTOR_GEARSET_06, false), rbD(1, E_MOTOR_GEARSET_06, true),
        flyWheel1(13, E_MOTOR_GEARSET_06, false),
        flyWheel2(16, E_MOTOR_GEARSET_06, true),
        turretMotor(6, E_MOTOR_GEARSET_06, true),
        intakeMotor(10, E_MOTOR_GEARSET_06, true), boomShackalacka({{22, 'B'}}),
        shoot3({{22, 'A'}}), shoot1({{22, 'C'}}), ejectPiston({{22, 'D'}}) {
    
    //setting PID values begining of motorcontrol class
    PID.driveFR.p = 4;
    PID.driveFR.i = 0.1;
    PID.driveFR.d = 5;

    PID.driveSS.p = 11;
    PID.driveSS.i = 0.065;
    PID.driveSS.d = 42.69;

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

  // class to set variables need to control moveTo function
  moveToInfoExternal_t move;

  //Function to rotate To a set angle
  double HeadingTarget = 0;
  void rotateTo(bool resetIntegs) {
    static moveToInfoInternal_t moveI;

    //setting integrals to 0 at start and reset
    static double IPIDSS = 0;
    static double previousets = 0;
    static bool finished = false;
    static bool resetMoveToSS = false;
    
    if (resetIntegs == true) {
      resetMoveToSS = true;
    }
    if (resetMoveToSS) {
      moveI.ets = 0;
      moveI.PIDSS = 0;
      moveI.PIDSSFLAT = 0;
      IPIDSS = 0;
      moveI.ets = 0;
    }

    //calculating proportional difference
    double currentheading = sensing.robot.angle;
    moveI.ets = HeadingTarget - currentheading;
    while (moveI.ets < -180) {
      moveI.ets += 360;
    }
    while (moveI.ets > 180) {
      moveI.ets -= 360;
    }

    //updating proportional
    IPIDSS += moveI.ets;

    //PID calculation
    moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                  PID.driveSS.d * (moveI.ets - previousets);

    //setting values for derivative calculation
    previousets = moveI.ets;

    // setting value inbetween limits of what motors can see
    moveI.PIDSSFLAT = moveI.PIDSS;
    if (moveI.PIDSSFLAT >= 2 * move.speed_limit) {
      moveI.PIDSSFLAT = 2 * move.speed_limit;
    }
    if (moveI.PIDSSFLAT <= -2 * move.speed_limit) {
      moveI.PIDSSFLAT = -2 * move.speed_limit;
    }

    //outputting speeds to turn at
    moveI.PIDSpeedR = moveI.PIDSSFLAT;
    moveI.PIDSpeedL = -moveI.PIDSSFLAT;
    if (fabs(moveI.ets) < move.errtheta &&
        fabs(lfD.get_actual_velocity()) + fabs(rfD.get_actual_velocity()) <
            40) {//angle is within range and speed is low enough
      resetMoveToSS = true;
      leftSpd = 0;
      rightSpd = 0;
    } else {//need to keep turning
      leftSpd = moveI.PIDSpeedL * 12000 / 127;
      rightSpd = moveI.PIDSpeedR * 12000 / 127;
    }
  }

  //function to travel to specific point
  void moveTo(bool resetIntegs) {
    //setting up variables and setting intially to 0
    static moveToInfoInternal_t moveI;
    static double IPIDSS = 0;
    static double previousets = 0;
    static double IPIDfw = 0;
    static double previouset = 0;
    double turnOrMoveMult = 1;

    //resetting variables when neccessary
    if (resetIntegs == true) {
      move.resetMoveTo = true;
    }
    if (move.resetMoveTo) {
      IPIDSS = 0;
      previousets = 0;
      IPIDfw = 0;
      previouset = 0;
      moveI.PIDSS = 0;     // PID turning speed
      moveI.PIDFW = 0;     // PID moveforward speed
      moveI.PIDSpeedL = 0; // PID leftside speed
      moveI.PIDSpeedR = 0; // PID rightside speed
      moveI.PIDFWFLAT = 0; // variable used for keeping move forward speed < 100
      moveI.PIDSSFLAT = 0; // variable used for keeping turning speed < 20
      // variable for for calculating first turning.
      move.resetMoveTo = false;
    }


    double currentheading = sensing.robot.angle / 180 * M_PI;
    /*
    function logic:
    find errors of position, turn to target if robot cannot move in a arc to
    it, than move to target in a arc. tracking center of robot is at the
    center of two tracking wheels do not recomand using this funciton with
    SpinTo() function. perferd to have a sperate thread for calculating live
    position, than just take out codes from line 41 to line 48
    */
    double etx = move.moveToxpos - sensing.robot.xpos; // change of x
    double ety = move.moveToypos - sensing.robot.ypos; // change of y
    double dist = sqrt(pow(etx, 2) + pow(ety, 2));
    double et = dist * 10;  //total distance needed to cover multiplied by constant to make it more pronounced

    //calculating necessary heading needed to go towards point and making sure that returned value is not undefined or impossible
    move.targetHeading = atan(ety / etx);
    if (etx < 0) {
      move.targetHeading += M_PI;
    } else if (etx == 0) {
      move.targetHeading = M_PI / 2 * (fabs(ety) / ety);
    }
    if (move.moveToforwardToggle == -1) {
      move.targetHeading += M_PI;
    }

    //calculating rotational difference and ensuring between +- 180
    moveI.ets = move.targetHeading - currentheading;
    if (moveI.ets < -M_PI) {
      moveI.ets += 2 * M_PI;
    }
    if (moveI.ets > M_PI) {
      moveI.ets -= 2 * M_PI;
    }

    //changing from radians to degreees
    moveI.ets = moveI.ets * 180 / M_PI;
    IPIDSS += moveI.ets;
    IPIDfw += et;

    //calculating turning PID
    moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                  PID.driveSS.d * (moveI.ets - previousets);

    //checking if withing range for no turning or only turning
    if (fabs(moveI.ets) < move.errtheta * turnOrMoveMult) {//if small enough distance to move forward
      if (fabs(moveI.ets - previousets) < 4) {//if slow enough reset turn integral and turning back on forward motion
        IPIDSS = 0;
        turnOrMoveMult = 1;
      }

      //calculating forward motion
      moveI.PIDFW = move.moveToforwardToggle *
                    (PID.driveFR.p * et + PID.driveFR.i * IPIDfw +
                      PID.driveFR.d * (et - previouset));
    } else {
      //slow motion down significantly to allow for greater influence of turning
      turnOrMoveMult = .2;
      moveI.PIDFW = 0;
    }

    //setting variables for derivative calculation 
    previousets = moveI.ets;
    previouset = et;

    // lowering speed of turning when moving slower so that it runs at the same percentage difference
    moveI.PIDSSFLAT = moveI.PIDSS * move.speed_limit / 100;
    moveI.PIDFWFLAT = moveI.PIDFW * move.speed_limit / 100;

    //making sure that speeds outputted do not exceed capabilities of the motor
    if (moveI.PIDFWFLAT >= move.speed_limit) {
      moveI.PIDFWFLAT = move.speed_limit;
    }
    if (moveI.PIDFWFLAT <= -move.speed_limit) {
      moveI.PIDFWFLAT = -move.speed_limit;
    }
    if (moveI.PIDSSFLAT >= 2 * move.speed_limit) {
      moveI.PIDSSFLAT = 2 * move.speed_limit;
    }
    if (moveI.PIDSSFLAT <= -2 * move.speed_limit) {
      moveI.PIDSSFLAT = -2 * move.speed_limit;
    }

    //changing from percent to millivolts for motor input
    rightSpd = (moveI.PIDFWFLAT + moveI.PIDSSFLAT) * 12000 / 127;
    leftSpd = (moveI.PIDFWFLAT - moveI.PIDSSFLAT) * 12000 / 127;

    //stopping and reseting varialbes 
    if (dist < move.tolerance) {
      move.resetMoveTo = true;
      leftSpd = 0;
      rightSpd = 0;
    }
  }

  //set pistions to correct starting position to ensure that there is no early expansion or shooting at start of match
  void setpistons(void) {
    shoot3.set_value(false);
    shoot1.set_value(false);
    ejectPiston.set_value(false);
    boomShackalacka.set_value(false);
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
        lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
      } else if (move.Stop_type == 0) {
        lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
        rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
      } else {
        lfD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        lbD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        rfD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
        rbD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
      }

      //driving or stopping the motor
      if (drivePowerL == 0 && drivePowerR == 0) {
        lfD.brake();
        lbD.brake();
        rfD.brake();
        rbD.brake();
      } else {
        lfD.move_voltage(leftSpd);
        lbD.move_voltage(leftSpd);
        rfD.move_voltage(rightSpd);
        rbD.move_voltage(rightSpd);
      }

      //outpuutting for match review
      logValue("x", sensing.robot.xpos, 0);
      logValue("y", sensing.robot.ypos, 1);
      logValue("ODOx", sensing.robot.odoxpos, 2);
      logValue("ODOy", sensing.robot.odoypos, 3);
      logValue("GPSx", sensing.robot.GPSxpos, 4);
      logValue("GPSy", sensing.robot.GPSypos, 5);
      logValue("heading", sensing.robot.angle, 6);
      logValue("turr ang", sensing.robot.turAng, 7);
      logValue("angDiff", angdiff, 8);
      logValue("batt pct", battery::get_capacity(), 9);
      logValue("lfdTemp", lfD.get_temperature(), 10);
      logValue("lbdTemp", lbD.get_temperature(), 11);
      logValue("rfdTemp", rfD.get_temperature(), 12);
      logValue("rbdTemp", rbD.get_temperature(), 13);
      logValue("intakeTemp", intakeMotor.get_temperature(), 14);
      logValue("turretTemp", turretMotor.get_temperature(), 15);
      logValue("fW1Temp", flyWheel1.get_temperature(), 16);
      logValue("fW2Temp", flyWheel2.get_temperature(), 17);
      logValue("fw1SPD", flyWheel1.get_actual_velocity(), 18);
      logValue("fw2SPD", flyWheel2.get_actual_velocity(), 19);
      logValue("magCount", sensing.robot.magFullness, 20);
      logValue("goalAngle", goalAngle, 21);
      logValue("goalSPD", sensing.goalSpeed, 22);
      logValue("goalSPD", angularVelocityCalc(sensing.goalSpeed), 23);
      //logging color from rollerGood
      logValue("rollerGood", sensing.rollerIsGood(),27);
      logValue("time", millis(), 28);

      delay(optimalDelay);
    }
    lfD.brake();
    lbD.brake();
    rfD.brake();
    rbD.brake();
  }

  // Operator control drive controller
  void driveController() {
    while (!competition::is_disabled()) {
      //setting variables and converting them to millivolts
      double leftSpdRaw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) +
                          master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      double rightSpdRaw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) -
                           master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

      leftSpd = leftSpdRaw * 12000 / 127;
      rightSpd = rightSpdRaw * 12000 / 127;

      lfD.move_voltage(leftSpd);
      lbD.move_voltage(leftSpd);
      rfD.move_voltage(rightSpd);
      rbD.move_voltage(rightSpd);

      //logging variables for match review
      logValue("x", sensing.robot.xpos, 0);
      logValue("y", sensing.robot.ypos, 1);
      logValue("ODOx", sensing.robot.odoxpos, 2);
      logValue("ODOy", sensing.robot.odoypos, 3);
      logValue("GPSx", sensing.robot.GPSxpos, 4);
      logValue("GPSy", sensing.robot.GPSypos, 5);
      logValue("heading", sensing.robot.angle, 6);
      logValue("turr ang", sensing.robot.turAng, 7);
      logValue("angDiff", angdiff, 8);
      logValue("batt pct", battery::get_capacity(), 9);
      logValue("lfdTemp", lfD.get_temperature(), 10);
      logValue("lbdTemp", lbD.get_temperature(), 11);
      logValue("rfdTemp", rfD.get_temperature(), 12);
      logValue("rbdTemp", rbD.get_temperature(), 13);
      logValue("intakeTemp", intakeMotor.get_temperature(), 14);
      logValue("turretTemp", turretMotor.get_temperature(), 15);
      logValue("fW1Temp", flyWheel1.get_temperature(), 16);
      logValue("fW2Temp", flyWheel2.get_temperature(), 17);
      logValue("fw1SPD", flyWheel1.get_actual_velocity(), 18);
      logValue("fw2SPD", flyWheel2.get_actual_velocity(), 19);
      logValue("magCount", sensing.robot.magFullness, 20);
      logValue("goalAngle", goalAngle, 21);
      logValue("goalSPD", sensing.goalSpeed, 22);
      logValue("goalSPD", angularVelocityCalc(sensing.goalSpeed), 23);
      logValue("rollerG", sensing.rollerIsGood(), 24);
      logValue("time", millis(), 25);

      delay(optimalDelay);
    }
  }

  // voltage controller for flywheel motors
  void flyController() {
    double maxSpeed = 610;

    // speed needed to accelerate
    double bottomLimit = 10;
    // speed needed to decelerate
    double topLimit = 30;

    // setting up PID controller values
    double angularVelocityDifferenceIntegral1 = 0;
    double angularVelocityDifferenceIntegral2 = 0;
    double prevAngularVelocityDifference1 = 0;
    double prevAngularVelocityDifference2 = 0;

    while (!competition::is_disabled()) {
      if (flyWheel1.get_temperature() < 45 &&
          flyWheel2.get_temperature() < 45) {
        double flyWVolt1, flyWVolt2;
        double goalAngularVelocity = angularVelocityCalc(sensing.goalSpeed);
        double holdPower1 = (18.48 * goalAngularVelocity) + 687.6;
        double holdPower2 = (17.57 * goalAngularVelocity) + 883.2;

        // positive values means acceleration is needed and negative means speed
        // is too high
        double angularVelocityDifference1 =
            goalAngularVelocity - flyWheel1.get_actual_velocity();
        double angularVelocityDifference2 =
            goalAngularVelocity - flyWheel2.get_actual_velocity();

        // checking nanf
        if (angularVelocityDifferenceIntegral1 == NAN) {
          angularVelocityDifferenceIntegral1 = 0;
        }
        if (angularVelocityDifferenceIntegral2 == NAN) {
          angularVelocityDifferenceIntegral2 = 0;
        }

        // flywheel1 controller
        if (angularVelocityDifference1 > bottomLimit) { // not fast enough
          angularVelocityDifferenceIntegral1 = 0;
          flyWVolt1 = 12000;
        } else if (angularVelocityDifference1 < -topLimit) { // too fast
          angularVelocityDifferenceIntegral1 = 0;
          flyWVolt1 = -12000;
        } else {
          double prop = angularVelocityDifference1 * PID.flyWheel.p * 0;
          double integ =
              angularVelocityDifferenceIntegral1 * PID.flyWheel.i * 0;
          if (!(integ > 60)) {
            angularVelocityDifferenceIntegral1 += angularVelocityDifference1;
          }
          double deriv =
              (prevAngularVelocityDifference2 - angularVelocityDifference2) *
              PID.flyWheel.d * 0;
          double PIDVAL = (prop + integ + deriv) * 12000 / 127 * 0;
          angularVelocityDifferenceIntegral1 += angularVelocityDifference1;
          flyWVolt1 = holdPower1 + PIDVAL;
          if (flyWVolt2 > 12000) {
            flyWVolt2 = 12000;
          }
          if (flyWVolt2 < -12000) {
            flyWVolt2 = -12000;
          }
        }

        // flywheel2 controller
        if (angularVelocityDifference2 > bottomLimit) { // not fast enough
          angularVelocityDifferenceIntegral2 = 0;
          flyWVolt2 = 12000;
        } else if (angularVelocityDifference2 < -topLimit) { // too fast
          angularVelocityDifferenceIntegral2 = 0;
          flyWVolt2 = -200;
        } else {
          double prop = angularVelocityDifference2 * PID.flyWheel.p * 0;
          double integ =
              angularVelocityDifferenceIntegral2 * PID.flyWheel.i * 0;
          if (!(integ > 60)) {
            angularVelocityDifferenceIntegral2 += angularVelocityDifference2;
          }
          double deriv =
              (prevAngularVelocityDifference2 - angularVelocityDifference2) *
              PID.flyWheel.d * 0;
          double PIDVAL = (prop + integ + deriv) * 12000 / 127 * 0;
          flyWVolt2 = holdPower2 + PIDVAL;
          if (flyWVolt2 > 12000) {
            flyWVolt2 = 12000;
          }
          if (flyWVolt2 < -12000) {
            flyWVolt2 = -12000;
          }
        }

        prevAngularVelocityDifference1 = angularVelocityDifference1;
        prevAngularVelocityDifference2 = angularVelocityDifference2;

        flyWheel1.move_voltage(flyWVolt1);
        flyWheel2.move_voltage(flyWVolt2);
      } else {
        static double IPIDang = 0;
        static double IPIDang2 = 0;

        //calculating how diffence in speed as input for PID and as tool to know when to shoot
        double flyWVolt;
        double flyWVolt2;
        double flyWheelW = flyWheel1.get_actual_velocity();
        double flyWheelW2 = flyWheel2.get_actual_velocity();
        diffFlyWheelW =
            angularVelocityCalc(sensing.robot.magFullness) - flyWheelW;
        diffFlyWheelW2 =
            angularVelocityCalc(sensing.robot.magFullness) - flyWheelW2;
        static double prevFWdiffSPD =
            angularVelocityCalc(sensing.robot.magFullness);
        static double prevFWdiffSPD2 =
            angularVelocityCalc(sensing.robot.magFullness);

        //updating Integral and derivative, calculating PID
        IPIDang += diffFlyWheelW;
        IPIDang2 += diffFlyWheelW2;
        double prop = PID.flyWheel.p * diffFlyWheelW;
        double prop2 = PID.flyWheel.p2 * diffFlyWheelW2;
        double integ = IPIDang * PID.flyWheel.i;
        double integ2 = IPIDang2 * PID.flyWheel.i2;
        double deriv = PID.flyWheel.d * (diffFlyWheelW - prevFWdiffSPD);
        double deriv2 = PID.flyWheel.d2 * (diffFlyWheelW2 - prevFWdiffSPD2);
        prevFWdiffSPD = diffFlyWheelW;
        prevFWdiffSPD2 = diffFlyWheelW2;


        //converting to millivolts and ensuring that it does not exceed the capabilities of the motors
        flyWVolt = 12000.0 / 127 * (prop + integ + deriv);
        flyWVolt2 = 12000.0 / 127 * (prop2 + integ2 + deriv2);

        if (flyWVolt > 12000) {
          flyWVolt = 12000;
        }
        if (flyWVolt < -12000) {
          flyWVolt = -12000;
        }
        if (fabs(diffFlyWheelW) < 1 && flyWVolt == 0) {
          IPIDang = 0;
        }
        if (flyWVolt2 > 12000) {
          flyWVolt2 = 12000;
        }
        if (flyWVolt2 < -12000) {
          flyWVolt2 = -12000;
        }
        if (fabs(diffFlyWheelW2) < 1 && flyWVolt2 == 0) {
          IPIDang2 = 0;
        }


        //checking for not a number returns, happens often when motor is disconnected
        if (isnanf(flyWVolt)) {
          flyWVolt = 0;
          prevFWdiffSPD = angularVelocityCalc(sensing.robot.magFullness);
          IPIDang = 0;
        }
        if (isnanf(flyWVolt2)) {
          flyWVolt2 = 0;
          prevFWdiffSPD2 = angularVelocityCalc(sensing.robot.magFullness);
          IPIDang2 = 0;
        }

        flyWheel1.move_voltage(flyWVolt);
        flyWheel2.move_voltage(flyWVolt2);
      }

      delay(optimalDelay);
    }
  }

  // power controller for that run intake and turret
  bool runTurretIntake = true;//variable to properly stop the thread
  void turretIntakeController() {
    runTurretIntake = true;
    while (!competition::is_disabled() && runTurretIntake == true) {

      turretMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
      if (!competition::is_autonomous()) {//control pistons in driver control
        if (master.get_digital(E_CONTROLLER_DIGITAL_A) &&//expand
            master.get_digital(E_CONTROLLER_DIGITAL_X) &&
            master.get_digital(E_CONTROLLER_DIGITAL_B) &&
            master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
          boomShackalacka.set_value(true);
          shoot3.set_value(false);
        } else {
          boomShackalacka.set_value(false); //operate 
          shoot1.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
          shoot3.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
          ejectPiston.set_value(
          master.get_digital(pros::E_CONTROLLER_DIGITAL_X));//operates piston to remove a disc stuck after flywheel
        }
      }

      double intakeSpd = intakeControl();
      double turrSpd = -turrControl() * 12000 / 127;
      if (turrSpd == 0) {
        turretMotor.brake();
      } else {
        turretMotor.move_voltage(turrSpd);
      }

      intakeMotor.move_voltage(intakeSpd);

      delay(optimalDelay);
    }
    turretMotor.brake();
    intakeMotor.brake();
  }

  // actuates the pistons during autonomous
  void raiseAScore(int number) {
    if (number == 3) {
      shoot3.set_value(true);
      delay(700);
      shoot3.set_value(false);
    } else {
      shoot1.set_value(true);
      delay(700);
      shoot1.set_value(false);
    }
  }

  // moves to roller and spins it to correct orientation
  void driveToRoller(double time,
                     bool reverseOut = true) { // changing to let move to take
                                               // over the initial moveto
    bool finished = false;
    driveType = 2;//manual voltage control

    int startTime = millis();
    while (!sensing.underRoller(1) && !sensing.underRoller(2) && //drive to roller until under roller
           millis() - startTime < time) {
      leftSpd = 8000;
      rightSpd = 8000;
      delay(20);
    }
    //run with constant power to ensure contact with roller
    leftSpd = 6000;
    rightSpd = 6000;

    intakeRunning = 3;//speed control for intake
    intakespdTarget = 180;
    while (finished == false && millis() - startTime < time) {//wait until roller is flipped
      static int pwr = 8000;
      if (sensing.rollerIsGood()) {
        finished = true;
        break;
      }
      delay(20);
    }

    if (reverseOut) {
      leftSpd = -8000;
      rightSpd = -8000;
      intakeRunning = 1;
      delay(300);
    }
    intakeRunning = 0;
  }

  // expansion
  void explode(void) {
    boomShackalacka.set_value(true);
    driveType = 2;
    rightSpd = 0;
    leftSpd = 0;
  }
};

//wrappers that exist to start threads inside a class
extern void drive_ControllerWrapper(void *mControl);
extern void turretIntake_ControllerWrapper(void *mControl);
extern void fly_ControllerWrapper(void *mControl);

#endif