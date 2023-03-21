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

  // basic class to handle each individual system
  class PID_t {
  public:
    double p, i, d, p2, i2, d2;
  };

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
      return sensing.goalSpeed * 1.5 + 30.842;
    } else if (number == 2 && discCountChoice == 2) {
      return sensing.goalSpeed * 1.30 + 3.805;
    } else {
      return sensing.goalSpeed * 1.14 + 30;
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

    static double PIDscalar = 5;
    static double gyroScalar = 0.2;
    static double chassisScalar = 0.35;
    static double turPredicScalar = .7;

    angdiff = goalAngle - sensing.robot.turAng;

    if (angdiff > 180) {
      angdiff -= 360;
    } else if (angdiff < -180) {
      angdiff += 360;
    }

    double robotAngleDiff = goalAngle - sensing.robot.angle;
    double turretAngle = double(sensing.turretEncoder.get_position()) / 100 +
                         180 - highTurretInitAng * 360;
    if (turretAngle + angdiff > 360) {
      angdiff -= 360;
    }
    if (turretAngle + angdiff < 0) {
      angdiff += 360;
    }
    updatedAD = true;

    static double previousveldiff = 0;
    static double previousangdiff = 0;

    if (!competition::is_disabled()) {
      if (fabs(angdiff) > 10) {
        double acceleration = .7;
        if (fabs(angdiff) / fabs(angdiff - previousangdiff) >
            fabs(angdiff - previousangdiff) / acceleration) { //accelerate
          if (angdiff < 0) {
            PIDPosition = -127;
          } else {
            PIDPosition = 127;
          }
          std::cout << fabs(angdiff - previousangdiff) << "\n";
        } else {
          if (angdiff < 0) {//deccelerate
            PIDPosition = 127;
          } else {
            PIDPosition = -127;
          }
          std::cout << fabs(angdiff - previousangdiff) << "\n";
        }
        previousangdiff = angdiff;
        IPIDposition = 0;
        IPIDvelocity = 0;
        return PIDPosition;

      } else if (fabs(angdiff) > 1){
        IPIDposition += angdiff;
        PIDPosition = (PID.turret.p * angdiff + PID.turret.i * IPIDposition +
                       PID.turret.d * (angdiff - previousangdiff));

        if (sensing.robot.turretLock && fabs(angdiff) < 5) {
          gyroScalar = 0;
          chassisScalar = 0;
          turPredicScalar = 0;
        } else {
          gyroScalar = 0.2;
          chassisScalar = 0.4;
          turPredicScalar = .7;
        }

        double veldiff = gyroScalar * T * (sensing.robot.angAccel) -
                         sensing.robot.velW * chassisScalar +
                         turPredicScalar * sensing.robot.turvelocity +
                         PIDPosition * PIDscalar;

        IPIDvelocity += veldiff;
        PIDVelocity = (1 * veldiff + 0.01 * IPIDvelocity +
                       0.1 * (veldiff - previousveldiff));
        previousveldiff = veldiff;
        if (fabs(angdiff) < 4 && PIDPosition == 0 && fabs(veldiff) < 1) {
          IPIDvelocity = 0;
        }
        if (fabs(veldiff) < 0.1) {
          IPIDvelocity = 0;
        }
        if (fabs(angdiff) < 3 && fabs(angdiff - previousangdiff) < 10) {
          return 0;
        }
        previousangdiff = angdiff;
      }
      else{
        return 0;
      }
    } else {
      PIDVelocity = 0;
      IPIDvelocity = 0;
      IPIDposition = 0;
    }

    if (fabs(PIDVelocity) < 0.01) {
      return 0;
    }
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
    static int baseSPD;
    static int jamTime = -9000;
    static int startTime = millis();
    static bool switched = false;
    if (master.get_digital(E_CONTROLLER_DIGITAL_R2) || intakeRunning == 1) {
      if (switched == false) {
        startTime = millis();
        switched = true;
      }
      if (millis() - jamTime > 500) {
        baseSPD = 12000;
      } else {
        baseSPD = -12000;
        startTime = millis();
      }
      if (millis() - startTime > 250 && millis() - jamTime > 500 &&
          fabs(intakeMotor.get_actual_velocity()) < 5) {
        jamTime = millis();
      }

    } else if (master.get_digital(E_CONTROLLER_DIGITAL_R1) ||
               intakeRunning == 2) {
      baseSPD = -12000;
      switched = false;

    } else if (intakeRunning == 3) {
      switched = false;
      if (intakeMotor.get_actual_velocity() < intakespdTar && baseSPD < 12000) {
        baseSPD += 100;
      } else if (intakeMotor.get_actual_velocity() > intakespdTar &&
                 baseSPD > -12000) {
        baseSPD -= 100;
      } else {
        baseSPD = baseSPD;
      }
    } else {
      baseSPD = 0;
      switched = false;
    }
    return baseSPD;
  }

public:
  bool updatedAD = false;
  double angdiff;
  int discCountChoice = 2; // 1 for single shot, 2 for automatic decision
  double intakespdTar = 0;
  int intakeRunning;
  double diffFlyWheelW;
  double diffFlyWheelW2;
  // Constructor to assign values to the motors and PID values
  motorControl_t(void)
      : lfD(5, E_MOTOR_GEARSET_06, true), lbD(4, E_MOTOR_GEARSET_06, false),
        rfD(2, E_MOTOR_GEARSET_06, false), rbD(1, E_MOTOR_GEARSET_06, true),
        flyWheel1(13, E_MOTOR_GEARSET_06, false),
        flyWheel2(16, E_MOTOR_GEARSET_06, true),
        turretMotor(6, E_MOTOR_GEARSET_06, true),
        intakeMotor(10, E_MOTOR_GEARSET_06, true), boomShackalacka({{22, 'B'}}),
        shoot3({{22, 'A'}}), shoot1({{22, 'C'}}), ejectPiston({{22, 'D'}}) {

    PID.driveFR.p = 4;
    PID.driveFR.i = 0.1;
    PID.driveFR.d = 5;

    PID.driveSS.p = 11;
    PID.driveSS.i = 0.065;
    PID.driveSS.d = 42.69;

    PID.turret.p = 3;
    PID.turret.i = .001;
    PID.turret.d = 10;

    PID.turret.p2 = 0.15; // 0.7
    PID.turret.i2 = 0.00000002;
    PID.turret.d2 = 14;

    PID.flyWheel.p = 1.82587;
    PID.flyWheel.i = 0.0219322;
    PID.flyWheel.d = 0.732158;
    PID.flyWheel.p2 = 1.84413;
    PID.flyWheel.i2 = 0.0221515;
    PID.flyWheel.d2 = 1.05928;
  }
  moveToInfoExternal_t move;
  double DistToTravel = 0;
  double HeadingTarget = 0;
  void driveDist(bool resetIntegs) {
    double currentheading = sensing.robot.angle * M_PI / 180;
    move.targetHeading = currentheading;
    lfD.tare_position();
    lbD.tare_position();
    rfD.tare_position();
    rbD.tare_position();
    moveToInfoInternal_t moveI;
    static double IPIDSS = 0;
    static double previousets = 0;
    static double IPIDfw = 0;
    static double previouset = 0;
    if (resetIntegs) {
      IPIDSS = 0;
      previousets = 0;
      IPIDfw = 0;
      previouset = 0;
    }
    moveI.dist = 0;      // change of position
    moveI.distR = 0;     // chagne of right postion
    moveI.distL = 0;     // change of left position
    moveI.PIDSS = 0;     // PID turning speed
    moveI.PIDFW = 0;     // PID moveforward speed
    moveI.PIDSpeedL = 0; // PID leftside speed
    moveI.PIDSpeedR = 0; // PID rightside speed
    moveI.PIDFWFLAT = 0; // variable used for keeping move forward speed < 100
    moveI.PIDSSFLAT = 0;
    double distTraveled = 0;
    currentheading = sensing.robot.angle * M_PI / 180;
    /*
    function logic:
    find errors of position, turn to target if robot cannot move in a arc to
    it, than move to target in a arc. tracking center of robot is at the
    center of two tracking wheels do not recomand using this funciton with
    SpinTo() function. perferd to have a sperate thread for calculating live
    position, than just take out codes from line 41 to line 48
    */
    double dist = DistToTravel - distTraveled;
    double et = dist * 10; ///////////////////////////////////////////////////////////////////////what
                           ///is this?

    moveI.ets = move.targetHeading - currentheading;
    if (moveI.ets < -M_PI) {
      moveI.ets += 2 * M_PI;
    }
    if (moveI.ets > M_PI) {
      moveI.ets -= 2 * M_PI;
    }

    moveI.ets = moveI.ets * 180 / M_PI;
    IPIDSS += moveI.ets;
    IPIDfw += et;
    if (move.moveToforwardToggle == 1) {
      moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                    PID.driveSS.d * (moveI.ets - previousets);
    } else {
      moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                    PID.driveSS.d * (moveI.ets - previousets);
    }
    if (fabs(moveI.ets) < move.errtheta) {
      if (move.moveToforwardToggle == 1) {
        moveI.PIDFW = move.moveToforwardToggle *
                      (PID.driveFR.p * et + PID.driveFR.i * IPIDfw +
                       PID.driveFR.d * (et - previouset));
      } else {
        moveI.PIDFW = move.moveToforwardToggle *
                      (PID.driveFR.p * et + PID.driveFR.i * IPIDfw +
                       PID.driveFR.d * (et - previouset));
      }
    } else {
      moveI.PIDFW = 0;
    }
    previousets = moveI.ets;
    previouset = et;
    moveI.PIDSSFLAT = moveI.PIDSS;
    moveI.PIDFWFLAT = moveI.PIDFW;
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
    if (move.moveToforwardToggle) {
      moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
      moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
    } else {
      moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
      moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
    }

    if (fabs(dist) < move.tolerance) {
      move.Stop_type = 1;
      leftSpd = 0;
      rightSpd = 0;
    } else {
      leftSpd = -moveI.PIDSpeedL;
      rightSpd = -moveI.PIDSpeedR;
    }
    distTraveled += double(lfD.get_position() + lbD.get_position() +
                           rfD.get_position() + rbD.get_position()) /
                    8 / 180 * M_PI * 1.375;
    lfD.tare_position();
    lbD.tare_position();
    rfD.tare_position();
    rbD.tare_position();
  }

  void rotateTo(bool resetIntegs) {
    double currentheading = sensing.robot.angle * M_PI / 180;
    moveToInfoInternal_t moveI;
    double IPIDSS = 0;
    double previousets = 0;
    double IPIDfw = 0;
    double previouset = 0;
    IPIDSS = 0;
    previousets = 0;
    IPIDfw = 0;
    previouset = 0;
    moveI.PIDSS = 0;     // PID turning speed
    moveI.PIDSpeedL = 0; // PID leftside speed
    moveI.PIDSpeedR = 0; // PID rightside speed
    moveI.PIDSSFLAT = 0;
    bool finished = false;
    bool resetMoveToSS = false;
    /*
    function logic:
    find errors of position, turn to target if robot cannot move in a arc to
    it, than move to target in a arc. tracking center of robot is at the
    center of two tracking wheels do not recomand using this funciton with
    SpinTo() function. perferd to have a sperate thread for calculating live
    position, than just take out codes from line 41 to line 48
    */
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
    currentheading = sensing.robot.angle;
    moveI.ets = HeadingTarget - currentheading;
    while (moveI.ets < -180) {
      moveI.ets += 360;
    }
    while (moveI.ets > 180) {
      moveI.ets -= 360;
    }
    IPIDSS += moveI.ets;

    moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                  PID.driveSS.d * (moveI.ets - previousets);

    previousets = moveI.ets;
    moveI.PIDSSFLAT = moveI.PIDSS;
    if (moveI.PIDSSFLAT >= 2 * move.speed_limit) {
      moveI.PIDSSFLAT = 2 * move.speed_limit;
    }
    if (moveI.PIDSSFLAT <= -2 * move.speed_limit) {
      moveI.PIDSSFLAT = -2 * move.speed_limit;
    }

    if (moveI.PIDSSFLAT > 127) {
      moveI.PIDSSFLAT = 127;
    }
    if (moveI.PIDSSFLAT < -127) {
      moveI.PIDSSFLAT = -127;
    }
    moveI.PIDSpeedR = moveI.PIDSSFLAT;
    moveI.PIDSpeedL = -moveI.PIDSSFLAT;
    if (fabs(moveI.ets) < move.errtheta &&
        fabs(lfD.get_actual_velocity()) + fabs(rfD.get_actual_velocity()) <
            40) {
      resetMoveToSS = true;
      leftSpd = 0;
      rightSpd = 0;
    } else {
      leftSpd = moveI.PIDSpeedL * 12000 / 127;
      rightSpd = moveI.PIDSpeedR * 12000 / 127;
    }
  }

  void moveTo(bool resetIntegs) {
    static moveToInfoInternal_t moveI;
    static double IPIDSS = 0;
    static double previousets = 0;
    static double IPIDfw = 0;
    static double previouset = 0;
    double turnOrMoveMult = 1;
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
    if (currentheading == PROS_ERR_F) {
      // JLO - handle error and exit, we can't continue
      std::cout << "\n headingFudge";
      return;
    }
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
    double et = dist * 10;

    move.targetHeading = atan(ety / etx);
    if (etx < 0) {
      move.targetHeading += M_PI;
    } else if (etx == 0) {
      move.targetHeading = M_PI / 2 * (fabs(ety) / ety);
    }
    if (move.moveToforwardToggle == -1) {
      move.targetHeading += M_PI;
    }
    moveI.ets = move.targetHeading - currentheading;
    if (moveI.ets < -M_PI) {
      moveI.ets += 2 * M_PI;
    }
    if (moveI.ets > M_PI) {
      moveI.ets -= 2 * M_PI;
    }

    moveI.ets = moveI.ets * 180 / M_PI;
    IPIDSS += moveI.ets;
    IPIDfw += et;
    if (move.moveToforwardToggle == 1) {
      moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                    PID.driveSS.d * (moveI.ets - previousets);
    } else {
      moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS +
                    PID.driveSS.d * (moveI.ets - previousets);
    }

    if (fabs(moveI.ets) < move.errtheta * turnOrMoveMult) {
      if (fabs(moveI.ets - previousets) < 4) {
        IPIDSS = 0;
        turnOrMoveMult = 1;
      }
      if (move.moveToforwardToggle == 1) {
        moveI.PIDFW = move.moveToforwardToggle *
                      (PID.driveFR.p * et + PID.driveFR.i * IPIDfw +
                       PID.driveFR.d * (et - previouset));
      } else {
        moveI.PIDFW = move.moveToforwardToggle *
                      (PID.driveFR.p * et + PID.driveFR.i * IPIDfw +
                       PID.driveFR.d * (et - previouset));
      }
    } else {
      turnOrMoveMult = .2;
      moveI.PIDFW = 0;
    }
    previousets = moveI.ets;
    previouset = et;
    moveI.PIDSSFLAT = moveI.PIDSS * move.speed_limit / 100;
    moveI.PIDFWFLAT = moveI.PIDFW * move.speed_limit / 100;
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
    rightSpd = (moveI.PIDFWFLAT + moveI.PIDSSFLAT) * 12000 / 127;
    leftSpd = (moveI.PIDFWFLAT - moveI.PIDSSFLAT) * 12000 / 127;
    if (dist < move.tolerance) {
      move.resetMoveTo = true;
      leftSpd = 0;
      rightSpd = 0;
    }
  }

  void setpistons(void) {
    shoot3.set_value(false);
    shoot1.set_value(false);
    ejectPiston.set_value(false);
    boomShackalacka.set_value(false);
  }

  void tailGater(bez_Return_t temp) {
    // rotateTo(atan((temp.returnPoints[0][1] - temp.returnPoints[1][1]) /
    // (temp.returnPoints[0][0] - temp.returnPoints[1][0])) * 180 / M_PI);
    bool finished = false;
    int current = 0;
    double errorCurveInteg = 0;
    double radiusScalar = 0;
    double radiusDifference = 9.45 / 2;

    // setting starting to nan so they get set to 0 by if statements
    double integralSS = NAN;
    double integralFW = NAN;
    double prevET = NAN;
    double prevETS = NAN;
    while (finished == false) {
      while (1) {
        double et =
            sqrt(pow(sensing.robot.xpos - temp.returnPoints[current][0], 2) +
                 pow(sensing.robot.ypos - temp.returnPoints[current][1], 2));

        if (et < 3 /*move.tolerance*/) { ///////////////land mine
          current++;
          if (current == temp.length + 1) {
            finished = true;
          }
          break; // path finished exit
        }
        // checking if closer to other point
        /*if (et > 6){
          double bET = et;
          for (int i = current; i<temp.length; i++){
            double tempet = sqrt(pow(sensing.robot.xpos -
        temp.returnPoints[i][0],2) + pow(sensing.robot.ypos -
        temp.returnPoints[i][1],2)); if (tempet < et){ current = i; et = tempet;
            }
          }
          if (bET != et){
            integralSS = 0;
            integralFW = 0;
          }
        }   */

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

  void waitPosTime(int maxTime, int overallStartTime,
                   int overallMaxTime = 55000) {
    int startTime = millis();
    move.resetMoveTo = false;
    while (millis() - startTime < maxTime && move.resetMoveTo == false &&
           millis() - overallStartTime < overallMaxTime) {
      delay(20);
    }
  }
  // 0 = moveTo, 1 = rotateTo, 2 = driveDist, 3 driveVoltage
  int driveType = 0;
  bool autoDriveRun = true;
  double leftSpd = 0;
  double rightSpd = 0;
  void autonDriveController(void) {
    autoDriveRun = true;
    while (!competition::is_disabled() && competition::is_autonomous() &&
           autoDriveRun == true) {
      static int prevdriveType = 1000;
      bool resetIntegs = false;
      if (driveType != prevdriveType) {
        resetIntegs = true;
      }
      switch (driveType) {
      case 0: // moveTo
        moveTo(resetIntegs);
        break;
      case 1: // rotateTo
        rotateTo(resetIntegs);
        break;
      case 2: // drivevDist
        driveDist(resetIntegs);
        break;
      case 3: // driveVoltage
        break;
      }
      prevdriveType = driveType;
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

      logValue("x", sensing.robot.xpos, 0);
      logValue("y", sensing.robot.ypos, 1);
      logValue("ODOx", sensing.robot.odoxpos, 2);
      logValue("ODOy", sensing.robot.odoypos, 3);
      logValue("GPSx", sensing.robot.GPSxpos, 4);
      logValue("GPSy", sensing.robot.GPSypos, 5);
      logValue("heading", sensing.robot.angle, 6);
      logValue("turr ang", sensing.robot.turAng, 7);
      logValue("batt pct", battery::get_capacity(), 8);
      logValue("lfdTemp", lfD.get_temperature(), 9);
      logValue("lbdTemp", lbD.get_temperature(), 10);
      logValue("rfdTemp", rfD.get_temperature(), 11);
      logValue("rbdTemp", rbD.get_temperature(), 12);
      logValue("intakeTemp", intakeMotor.get_temperature(), 13);
      logValue("turretTemp", turretMotor.get_temperature(), 14);
      logValue("fW1Temp", flyWheel1.get_temperature(), 15);
      logValue("fW2Temp", flyWheel2.get_temperature(), 16);
      logValue("fw1SPD", flyWheel1.get_actual_velocity(), 17);
      logValue("fw2SPD", flyWheel2.get_actual_velocity(), 18);
      logValue("magCount", sensing.robot.magFullness, 19);
      logValue("goalAngle", goalAngle, 20);
      logValue("goalSPD", sensing.goalSpeed, 21);
      logValue("goalSPD", angularVelocityCalc(sensing.goalSpeed), 22);
      logValue("time", millis(), 23);

      delay(optimalDelay);
    }
    lfD.brake();
    lbD.brake();
    rfD.brake();
    rbD.brake();
  }

  void driveController() {
    while (!competition::is_disabled()) {
      double leftSpd;
      double rightSpd;
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

      logValue("x", sensing.robot.xpos, 0);
      logValue("y", sensing.robot.ypos, 1);
      logValue("ODOx", sensing.robot.odoxpos, 2);
      logValue("ODOy", sensing.robot.odoypos, 3);
      logValue("GPSx", sensing.robot.GPSxpos, 4);
      logValue("GPSy", sensing.robot.GPSypos, 5);
      logValue("heading", sensing.robot.angle, 6);
      logValue("turret Angle", sensing.robot.turAng, 7);
      logValue("battery Level", battery::get_capacity(), 8);
      logValue("lfdTemp", lfD.get_temperature(), 9);
      logValue("lbdTemp", lbD.get_temperature(), 10);
      logValue("rfdTemp", rfD.get_temperature(), 11);
      logValue("rbdTemp", rbD.get_temperature(), 12);
      logValue("intakeTemp", intakeMotor.get_temperature(), 13);
      logValue("turretTemp", turretMotor.get_temperature(), 14);
      logValue("flyWheel1Temp", flyWheel1.get_temperature(), 15);
      logValue("flyWheel2Temp", flyWheel2.get_temperature(), 16);
      logValue("flyWheel1SPD", flyWheel1.get_actual_velocity(), 17);
      logValue("flyWheel2SPD", flyWheel2.get_actual_velocity(), 18);
      logValue("magFullness", sensing.robot.magFullness, 19);
      logValue("goalAngle", goalAngle, 20);
      logValue("goalSPD", sensing.goalSpeed, 21);
      logValue("goalSPD", angularVelocityCalc(sensing.goalSpeed), 22);
      logValue("time", millis(), 23);

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
          flyWVolt1 = -300;
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
        if (competition::is_disabled()) {
          IPIDang = 0;
          IPIDang2 = 0;
        }
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

  // power controller for differential motors between intake and turret
  bool runTurretIntake = true;
  void turretIntakeController() {
    runTurretIntake = true;
    while (!competition::is_disabled() && runTurretIntake == true) {

      turretMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
      if (!competition::is_autonomous()) {
        ejectPiston.set_value(
            master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
        if (master.get_digital(E_CONTROLLER_DIGITAL_A) &&
            master.get_digital(E_CONTROLLER_DIGITAL_X) &&
            master.get_digital(E_CONTROLLER_DIGITAL_B) &&
            master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
          boomShackalacka.set_value(true);
          shoot3.set_value(false);
        } else {
          boomShackalacka.set_value(false);
          shoot1.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
          shoot3.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
          ejectPiston.set_value(
              master.get_digital(pros::E_CONTROLLER_DIGITAL_X));
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

  void raiseAScore(int number) {
    if (number == 3) {
      shoot3.set_value(true);
      delay(300);
      shoot3.set_value(false);
    } else {
      shoot1.set_value(true);
      delay(300);
      shoot1.set_value(false);
    }
  }

  void driveToRoller(double time,
                     bool reverseOut = true) { // changing to let move to take
                                               // over the initial moveto
    bool finished = false;
    driveType = 3;
    int startTime = millis();
    while (!sensing.underRoller(1) && !sensing.underRoller(2) &&
           millis() - startTime < time) {
      leftSpd = 8000;
      rightSpd = 8000;
      delay(20);
    }
    leftSpd = 6000;
    rightSpd = 6000;
    intakeRunning = 3;
    intakespdTar = 180;
    while (finished == false && millis() - startTime < time) {
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
    driveType = 3;
    rightSpd = 0;
    leftSpd = 0;
  }
};

extern void drive_ControllerWrapper(void *mControl);
extern void turretIntake_ControllerWrapper(void *mControl);
extern void fly_ControllerWrapper(void *mControl);

#endif