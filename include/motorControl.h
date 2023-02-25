#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "api.h"
#include "pros/misc.hpp"
#include "sdLogging.h"
#include "Autons/autonSetup.h"
#include "robotConfig.h"
#include "devFuncs.h"
#include "bezierCalculations.h"

using namespace pros;

class motorControl_t
{
private:
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
    int optimalDelay = 20;

    class PID_t
    {
    public:
        double p, i, d, p2, i2, d2;
    };
    class tunedSystems_t
    {
    public:
        PID_t driveFR, driveSS, turret, flyWheel;
    } PID;

    class moveToInfoInternal_t
    {
    public:
        double moveToxpos, moveToypos, targetHeading, ets, speed_limit = 100, errtheta = 5;
        double dist = 0;      // change of position
        double distR = 0;     // chagne of right postion
        double distL = 0;     // change of left position
        double PIDSS = 0;     // PID turning speed
        double PIDFW = 0;     // PID moveforward speed
        double PIDSpeedL = 0; // PID leftside speed
        double PIDSpeedR = 0; // PID rightside speed
        double prevPIDFW = 0; // PID moveforward speed at t = -1
        double prevPIDSS = 0; // PID turning speed at t = -1
        double PIDFWFLAT = 0; // variable used for keeping move forward speed < 100
        double PIDSSFLAT = 0;
        int moveToforwardToggle = 1, Stop_type = 2;
        double tolerance = 2;
    };

    class moveToInfoExternal_t
    {
    public:
        double moveToxpos, moveToypos, targetHeading, speed_limit = 100, errtheta = 5;
        int moveToforwardToggle = 1, Stop_type = 2;
        double tolerance = 5;
        bool resetMoveTo;
    };

    void moveTo(double &leftSpd, double &rightSpd)
    {
        static moveToInfoInternal_t moveI;

        static double IPIDSS = 0;
        static double previousets = 0;
        static double IPIDfw = 0;
        static double previouset = 0;
        if (move.resetMoveTo)
        {
            IPIDSS = 0;
            previousets = 0;
            IPIDfw = 0;
            previouset = 0;
            moveI.dist = 0;      // change of position
            moveI.distR = 0;     // chagne of right postion
            moveI.distL = 0;     // change of left position
            moveI.PIDSS = 0;     // PID turning speed
            moveI.PIDFW = 0;     // PID moveforward speed
            moveI.PIDSpeedL = 0; // PID leftside speed
            moveI.PIDSpeedR = 0; // PID rightside speed
            moveI.prevPIDFW = 0; // PID moveforward speed at t = -1
            moveI.prevPIDSS = 0; // PID turning speed at t = -1
            moveI.PIDFWFLAT = 0; // variable used for keeping move forward speed < 100
            moveI.PIDSSFLAT = 0; // variable used for keeping turning speed < 20
            // variable for for calculating first turning.
            move.resetMoveTo = false;
        }
        double currentheading = sensing.robot.angle / 180 * M_PI;
        if (currentheading == PROS_ERR_F)
        {
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

        // std::cout << "\n Hi5";
        move.targetHeading = atan(ety / etx);
        if (etx < 0)
        {
            move.targetHeading += M_PI;
        }
        else if (etx == 0)
        {
            move.targetHeading = M_PI / 2 * (fabs(ety) / ety);
        }
        if (move.moveToforwardToggle == -1)
        {
            move.targetHeading += M_PI;
        }
        moveI.ets = move.targetHeading - currentheading;
        if (moveI.ets < -M_PI)
        {
            moveI.ets += 2 * M_PI;
        }
        if (moveI.ets > M_PI)
        {
            moveI.ets -= 2 * M_PI;
        }

        moveI.ets = moveI.ets * 180 / M_PI;
        IPIDSS += moveI.ets;
        IPIDfw += et;
        if (move.moveToforwardToggle == 1)
        {
            moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
        }
        else
        {
            moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
        }
        if (fabs(moveI.ets) < 10)
        {
            if (move.moveToforwardToggle == 1)
            {
                moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
            }
            else
            {
                moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
            }
        }
        else
        {
            moveI.PIDFW = 0;
        }
        previousets = moveI.ets;
        previouset = et;
        moveI.PIDSSFLAT = moveI.PIDSS;
        moveI.PIDFWFLAT = moveI.PIDFW;
        if (moveI.PIDFWFLAT >= move.speed_limit)
        {
            moveI.PIDFWFLAT = move.speed_limit;
        }
        if (moveI.PIDFWFLAT <= -move.speed_limit)
        {
            moveI.PIDFWFLAT = -move.speed_limit;
        }
        if (moveI.PIDSSFLAT >= 2 * move.speed_limit)
        {
            moveI.PIDSSFLAT = 2 * move.speed_limit;
        }
        if (moveI.PIDSSFLAT <= -2 * move.speed_limit)
        {
            moveI.PIDSSFLAT = -2 * move.speed_limit;
        }
        if (move.moveToforwardToggle)
        {
            moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
            moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
        }
        else
        {
            moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
            moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
        }
        if (dist < move.tolerance)
        {
            move.resetMoveTo = true;
            if (move.Stop_type == 1)
            {
                // motor stop (hold)
                leftSpd = 0;
                rightSpd = 0;
                lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
            }
            else
            {
                // motor stop (coast)
                leftSpd = 0;
                rightSpd = 0;
                lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
            }
        }
        else
        {
            leftSpd = -moveI.PIDSpeedL;
            rightSpd = -moveI.PIDSpeedR;
        }
    }

    double angularVelocityCalc(void)
    {
        // return master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) + 250;
        return sensing.goalSpeed * 1.56;
    }

    bool recoilPrevent;
     double turrControl(void)
    {
        static double PIDPosition = 0;
        static double PIDVelocity = 0;
        static double T = 0;
        static double previousT = 0;
        static double PIDscalar = 1;
        static double gyroScalar = 0.1;
        static double chassisScalar = 0.35;//0.3;
        static double turPredicScalar = 1;
        double angdiff;
        T = float(millis()) / 1000 - previousT;
        previousT += T;
        /*if (!competition::is_disabled() && !competition::is_autonomous()){
            robotGoal.angleBetweenHorABS = robot.angle + 180;
        }*/

        angdiff = goalAngle - sensing.robot.turAng;

        if (angdiff > 180)
        {
            angdiff -= 360;
        }
        else if (angdiff < -180)
        {
            angdiff += 360;
        }
        if (fabs(angdiff) < 2)
        {
            angdiff = 0;
        }

        static double IPIDvel = 0;
        static double previousveldiff = 0;
        static double IPIDang = 0;
        static double previousangdiff = 0;
        if (!competition::is_disabled())
        {   
            IPIDang += angdiff;
            PIDPosition = (PID.turret.p * angdiff + PID.turret.i * IPIDang + PID.turret.d * (angdiff - previousangdiff));
            if (sensing.robot.turretLock &&fabs(angdiff) < 5){
                gyroScalar = 0;
                chassisScalar = 0;//0.3;
                turPredicScalar =0;
                
            }
            else{
            gyroScalar = 0.1;
            chassisScalar = 0.35;//0.3;
            turPredicScalar = 1;
            }
            //std::cout << "\n" << goalAngle;
            //std::cout << "\n x:" << sensing.robot.GPSxpos<<" y:"<<sensing.robot.GPSypos << " turretctl:" <<sensing.robot.turvelocity;
            //std::cout << "\ntposition correction:" << PIDPosition;
            //std::cout << "\nangAccel         :" << sensing.robot.angAccel;

            previousangdiff = angdiff;
            double veldiff = gyroScalar * T * (sensing.robot.angAccel) 
            - sensing.robot.velW * chassisScalar 
            + turPredicScalar * sensing.robot.turvelocity 
            + PIDPosition * PIDscalar -sensing.robot.turvelw ;
            IPIDvel += veldiff;
            //std::cout<<"\n"<<veldiff;
            PIDVelocity = (1 * veldiff + 0.01 * IPIDvel + 0.1 * (veldiff - previousveldiff));
            previousveldiff = veldiff;
            if (fabs(angdiff) < 1 && PIDPosition == 0 && fabs(veldiff)<1)
            {
                IPIDang = 0;
            }
            if (fabs(veldiff) < 0.1)
            {
                IPIDvel = 0;
            }
        }
        else
        {
            PIDVelocity = 0;
        }
        if (fabs(PIDVelocity) < 0.01)
        {
            return 0;
        }
        if (isnanf(PIDVelocity))
        {
            PIDVelocity = 0;
            IPIDvel = 0;
            IPIDang = 0;
        }

        return PIDVelocity;
    }

    double intakeControl(void)
    {
        int baseSPD;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2) || intakeRunning == 1)
        {
            baseSPD = 127;
        }
        else if (master.get_digital(E_CONTROLLER_DIGITAL_R1) || intakeRunning == 2)
        {
            baseSPD = -127;
        }
        else
        {
            baseSPD = 0;
        }

        return baseSPD;
    }

public:
    int intakeRunning;
    bool spinRoller = 0;
    // Constructor to assign values to the motors and PID values
    motorControl_t(void) : lfD(5, E_MOTOR_GEARSET_06, true), lbD(4, E_MOTOR_GEARSET_06, false), rfD(2, E_MOTOR_GEARSET_06, false), rbD(1, E_MOTOR_GEARSET_06, true),
                           flyWheel1(13, E_MOTOR_GEARSET_06, false), flyWheel2(17, E_MOTOR_GEARSET_06, true),
                           turretMotor(6, E_MOTOR_GEARSET_06, true), intakeMotor(10, E_MOTOR_GEARSET_06, true), boomShackalacka({{22, 'B'}}), shoot3({{22, 'A'}}), shoot1({{22, 'C'}}), ejectPiston({{22, 'D'}})
    {
        PID.driveFR.p = .7;
        PID.driveFR.i = 0;
        PID.driveFR.d = 0;

        PID.driveSS.p = .00;
        PID.driveSS.i = 0;
        PID.driveSS.d = 0.0;

        PID.turret.p = 0.8;
        PID.turret.i = .02;
        PID.turret.d = 27;

        PID.turret.p2 = 0.7;
        PID.turret.i2 = 0.0002;
        PID.turret.d2 = 0;

        PID.flyWheel.p = 1.82587;
        PID.flyWheel.i = 0.0219322;
        PID.flyWheel.d = 0.732158;
        PID.flyWheel.p2 = 1.84413;
        PID.flyWheel.i2 = 0.0221515;
        PID.flyWheel.d2 = 1.05928;
    }
    moveToInfoExternal_t move;
    void driveDist(double goalDist, int P = -100, int I = -100, int D = -100)
    {
        if (P != -100)
        {
            PID.driveFR.p = P;
            PID.driveFR.i = I;
            PID.driveFR.d = D;
        }
        double currentheading = sensing.robot.angle * M_PI / 180;
        move.targetHeading = currentheading;
        lfD.tare_position();
        lbD.tare_position();
        rfD.tare_position();
        rbD.tare_position();
        moveToInfoInternal_t moveI;
        double IPIDSS = 0;
        double previousets = 0;
        double IPIDfw = 0;
        double previouset = 0;
        IPIDSS = 0;
        previousets = 0;
        IPIDfw = 0;
        previouset = 0;
        moveI.dist = 0;      // change of position
        moveI.distR = 0;     // chagne of right postion
        moveI.distL = 0;     // change of left position
        moveI.PIDSS = 0;     // PID turning speed
        moveI.PIDFW = 0;     // PID moveforward speed
        moveI.PIDSpeedL = 0; // PID leftside speed
        moveI.PIDSpeedR = 0; // PID rightside speed
        moveI.prevPIDFW = 0; // PID moveforward speed at t = -1
        moveI.prevPIDSS = 0; // PID turning speed at t = -1
        moveI.PIDFWFLAT = 0; // variable used for keeping move forward speed < 100
        moveI.PIDSSFLAT = 0;
        move.resetMoveTo = false;
        double distTraveled = 0;
        while (!move.resetMoveTo)
        {

            currentheading = sensing.robot.angle * M_PI / 180;
            /*
            function logic:
            find errors of position, turn to target if robot cannot move in a arc to
            it, than move to target in a arc. tracking center of robot is at the
            center of two tracking wheels do not recomand using this funciton with
            SpinTo() function. perferd to have a sperate thread for calculating live
            position, than just take out codes from line 41 to line 48
            */
            double dist = goalDist - distTraveled;
            double et = dist * 10;

            moveI.ets = move.targetHeading - currentheading;
            if (moveI.ets < -M_PI)
            {
                moveI.ets += 2 * M_PI;
            }
            if (moveI.ets > M_PI)
            {
                moveI.ets -= 2 * M_PI;
            }

            moveI.ets = moveI.ets * 180 / M_PI;
            IPIDSS += moveI.ets;
            IPIDfw += et;
            if (move.moveToforwardToggle == 1)
            {
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            else
            {
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            if (fabs(moveI.ets) < 10)
            {
                if (move.moveToforwardToggle == 1)
                {
                    moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }
                else
                {
                    moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }
            }
            else
            {
                moveI.PIDFW = 0;
            }
            previousets = moveI.ets;
            previouset = et;
            moveI.PIDSSFLAT = moveI.PIDSS;
            moveI.PIDFWFLAT = moveI.PIDFW;
            if (moveI.PIDFWFLAT >= move.speed_limit)
            {
                moveI.PIDFWFLAT = move.speed_limit;
            }
            if (moveI.PIDFWFLAT <= -move.speed_limit)
            {
                moveI.PIDFWFLAT = -move.speed_limit;
            }
            if (moveI.PIDSSFLAT >= 2 * move.speed_limit)
            {
                moveI.PIDSSFLAT = 2 * move.speed_limit;
            }
            if (moveI.PIDSSFLAT <= -2 * move.speed_limit)
            {
                moveI.PIDSSFLAT = -2 * move.speed_limit;
            }
            if (move.moveToforwardToggle)
            {
                moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
                moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
            }
            else
            {
                moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
                moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
            }

            if (fabs(dist) < move.tolerance)
            {
                move.resetMoveTo = true;
                move.Stop_type = 1;
                if (move.Stop_type == 1)
                {
                    // motor stop (hold)
                    lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                }
                else
                {
                    // motor stop (coast)
                    lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                }
                lfD.brake();
                rfD.brake();
                lbD.brake();
                rbD.brake();
            }
            else
            {
                lfD.move(-moveI.PIDSpeedL);
                rfD.move(-moveI.PIDSpeedR);
                lbD.move(-moveI.PIDSpeedL);
                rbD.move(-moveI.PIDSpeedR);
            }
            distTraveled += double(lfD.get_position() + lbD.get_position() + rfD.get_position() + rbD.get_position()) / 8 / 180 * M_PI * 1.375;
            lfD.tare_position();
            lbD.tare_position();
            rfD.tare_position();
            rbD.tare_position();
            delay(20);
        }
        lfD.brake();
        rfD.brake();
        lbD.brake();
        rbD.brake();
    }

    void rotateTo(double angleTo, int P = -100, int I = -100, int D = -100)
    {
        if (P != -100)
        {
            PID.driveSS.p = P;
            PID.driveSS.i = I;
            PID.driveSS.d = D;
        }
        double currentheading = sensing.robot.angle * M_PI / 180;
        move.targetHeading = currentheading;
        moveToInfoInternal_t moveI;
        double IPIDSS = 0;
        double previousets = 0;
        double IPIDfw = 0;
        double previouset = 0;
        IPIDSS = 0;
        previousets = 0;
        IPIDfw = 0;
        previouset = 0;
        moveI.dist = 0;      // change of position
        moveI.distR = 0;     // chagne of right postion
        moveI.distL = 0;     // change of left position
        moveI.PIDSS = 0;     // PID turning speed
        moveI.PIDSpeedL = 0; // PID leftside speed
        moveI.PIDSpeedR = 0; // PID rightside speed
        moveI.prevPIDSS = 0; // PID turning speed at t = -1
        moveI.PIDSSFLAT = 0;
        bool resetMoveToSS = false;
        angleTo = angleTo * M_PI / 180;
        while (!resetMoveToSS)
        {
            /*
            function logic:
            find errors of position, turn to target if robot cannot move in a arc to
            it, than move to target in a arc. tracking center of robot is at the
            center of two tracking wheels do not recomand using this funciton with
            SpinTo() function. perferd to have a sperate thread for calculating live
            position, than just take out codes from line 41 to line 48
            */

            currentheading = sensing.robot.angle * M_PI / 180;
            moveI.ets = angleTo - currentheading;
            if (moveI.ets < -M_PI)
            {
                moveI.ets += 2 * M_PI;
            }
            if (moveI.ets > M_PI)
            {
                moveI.ets -= 2 * M_PI;
            }

            moveI.ets = moveI.ets * 180 / M_PI;
            IPIDSS += moveI.ets;
            if (move.moveToforwardToggle == 1)
            {
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            else
            {
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }

            if (moveI.ets > 180)
            {
                moveI.PIDSS *= -1;
            }
            previousets = moveI.ets;
            moveI.PIDSSFLAT = moveI.PIDSS;
            if (moveI.PIDSSFLAT >= 2 * move.speed_limit)
            {
                moveI.PIDSSFLAT = 2 * move.speed_limit;
            }
            if (moveI.PIDSSFLAT <= -2 * move.speed_limit)
            {
                moveI.PIDSSFLAT = -2 * move.speed_limit;
            }

            if (moveI.PIDSSFLAT > 127)
            {
                moveI.PIDSSFLAT = 127;
            }
            if (moveI.PIDSSFLAT < -127)
            {
                moveI.PIDSSFLAT = -127;
            }

            if (fabs(moveI.ets) < 15)
            {
                if (moveI.ets > 0)
                {
                    moveI.PIDSSFLAT = 50;
                }
                else
                {
                    moveI.PIDSSFLAT = -50;
                }
            }

            moveI.PIDSpeedR = -moveI.PIDSSFLAT;
            moveI.PIDSpeedL = moveI.PIDSSFLAT;

            if (fabs(moveI.ets) < 2 && lfD.get_actual_velocity() < 40)
            {
                resetMoveToSS = true;
                move.Stop_type = 1;
                if (move.Stop_type == 1)
                {
                    // motor stop (hold)
                    lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                }
                else
                {
                    // motor stop (coast)
                    lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                }
            }
            else
            {
                lfD.move_voltage(-moveI.PIDSpeedL * 12000 / 127);
                rfD.move_voltage(-moveI.PIDSpeedR * 12000 / 127);
                lbD.move_voltage(-moveI.PIDSpeedL * 12000 / 127);
                rbD.move_voltage(-moveI.PIDSpeedR * 12000 / 127);
            }
            delay(20);
        }
        lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        lfD.brake();
        rfD.brake();
        lbD.brake();
        rbD.brake();
    }

    void setpistons(void)
    {
        shoot3.set_value(false);
        shoot1.set_value(false);
        ejectPiston.set_value(false);
        boomShackalacka.set_value(false);
    }

    void tailGater(bez_Return_t temp){
        double radiusDifference = 9.5;
        double radiusScalar = 1;
        int current = 0;
        double IPIDSS = 0;
        double IPIDfw = 0;
        double previousets = 0;
        double previouset = 0;
        while(current < temp.length){
            moveToInfoInternal_t moveI;
            double dist = sqrt(pow(sensing.robot.xpos - temp.returnPoints[current][0], 2) + pow(sensing.robot.ypos - temp.returnPoints[current][1], 2));
            while(dist > move.tolerance){
                /*
                function logic:
                find errors of position, turn to target if robot cannot move in a arc to
                it, than move to target in a arc. tracking center of robot is at the
                center of two tracking wheels do not recomand using this funciton with
                SpinTo() function. perferd to have a sperate thread for calculating live
                position, than just take out codes from line 41 to line 48
                */
                double currentheading = sensing.robot.angle / 180 * M_PI;
                double etx = temp.returnPoints[current][0] - sensing.robot.xpos; // change of x
                double ety = temp.returnPoints[current][1] - sensing.robot.ypos; // change of y
                std::cout << "dx: " << etx << ", dy: " << ety <<"\n";
                std::cout << "x: " << sensing.robot.xpos << ", y: " << sensing.robot.ypos <<"\n";
                //logValue("etx", etx, 7);
                //logValue("ety", ety, 8);
                //outValsSDCard();
                double distance;
                if (temp.length - 1 != current){
                    distance = sqrt(pow(etx, 2) + pow(ety, 2));
                }
                else{
                    distance = 24;
                }
                double et = distance * 10;

                move.targetHeading = atan(ety / etx);
                if (etx < 0)
                {
                    move.targetHeading += M_PI;
                }
                else if (etx == 0)
                {
                    move.targetHeading = M_PI / 2 * (fabs(ety) / ety);
                }
                if (move.moveToforwardToggle == -1)
                {
                    move.targetHeading += M_PI;
                }
                moveI.ets = move.targetHeading - currentheading;
                if (moveI.ets < -M_PI)
                {
                    moveI.ets += 2 * M_PI;
                }
                if (moveI.ets > M_PI)
                {
                    moveI.ets -= 2 * M_PI;
                }

                moveI.ets = moveI.ets * 180 / M_PI;
                IPIDSS += moveI.ets;
                IPIDfw += et;
                if (move.moveToforwardToggle == 1)
                {
                    moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
                }
                else
                {
                    moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
                }

                if (move.moveToforwardToggle == 1)
                {
                    moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }
                else
                {
                    moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }

                previousets = moveI.ets;
                previouset = et;
                moveI.PIDSSFLAT = moveI.PIDSS;
                moveI.PIDFWFLAT = moveI.PIDFW;
                if (moveI.PIDFWFLAT >= move.speed_limit)
                {
                    moveI.PIDFWFLAT = move.speed_limit;
                }
                if (moveI.PIDFWFLAT <= -move.speed_limit)
                {
                    moveI.PIDFWFLAT = -move.speed_limit;
                }
                if (moveI.PIDSSFLAT >= 2 * move.speed_limit)
                {
                    moveI.PIDSSFLAT = 2 * move.speed_limit;
                }
                if (moveI.PIDSSFLAT <= -2 * move.speed_limit)
                {
                    moveI.PIDSSFLAT = -2 * move.speed_limit;
                }
                double radIn;
                double radOut;

                if (temp.radius[current] != 420.69){
                    radIn = (temp.radius[current]-radiusDifference)*radiusScalar;
                    radOut = (temp.radius[current]+radiusDifference)*radiusScalar;
                }
                else{
                    radIn = 1;
                    radOut = 1;
                }

                if (moveI.PIDFWFLAT*radOut/radIn > 127){
                    moveI.PIDFWFLAT = moveI.PIDFWFLAT*radIn/radOut;
                }
                
                if (moveI.ets >0){
                    moveI.PIDSpeedL = moveI.PIDFWFLAT*radIn/radOut;
                    moveI.PIDSpeedR = moveI.PIDFWFLAT*radOut/radIn;
                }else{
                    moveI.PIDSpeedR = moveI.PIDFWFLAT*radIn/radOut;
                    moveI.PIDSpeedL = moveI.PIDFWFLAT*radOut/radIn;  
                }
                moveI.PIDSpeedR += moveI.PIDSSFLAT;
                moveI.PIDSpeedL -= moveI.PIDSSFLAT;

                if (moveI.PIDSpeedL > 127)
                {
                    moveI.PIDSpeedL = 127;
                }
                if (moveI.PIDSpeedR > 127)
                {
                    moveI.PIDSpeedR = 127;
                }

                if (moveI.PIDSpeedL < -127)
                {
                    moveI.PIDSpeedL = -127;
                }
                if (moveI.PIDSpeedR < -127)
                {
                    moveI.PIDSpeedR = -127;
                }
                dist = sqrt(pow(sensing.robot.xpos - temp.returnPoints[current][0], 2) + pow(sensing.robot.ypos - temp.returnPoints[current][1], 2));
                lfD.move(moveI.PIDSpeedL);
                lbD.move(moveI.PIDSpeedL);
                rfD.move(moveI.PIDSpeedR);
                rbD.move(moveI.PIDSpeedR);
                delay(20);
            }
            current++;
            delay(20);
        }
        
        lfD.brake();
        lbD.brake();
        rfD.brake();
        rbD.brake();
    }

    void waitPosTime(int maxTime)
    {
        int startTime = millis();
        move.resetMoveTo = false;
        while (millis() - startTime < maxTime && move.resetMoveTo == false)
        {
            delay(20);
        }
    }

    void autonDriveController(void)
    {
        while (!competition::is_disabled() && competition::is_autonomous())
        {

            static double leftSpd = 0;
            static double rightSpd = 0;

            moveTo(leftSpd, rightSpd);
            if (leftSpd > 127)
            {
                leftSpd = 127;
            }
            if (rightSpd > 127)
            {
                rightSpd = 127;
            }

            if (leftSpd < -127)
            {
                leftSpd = -127;
            }
            if (rightSpd < -127)
            {
                rightSpd = -127;
            }

            lfD.move(leftSpd);
            lbD.move(leftSpd);
            rfD.move(rightSpd);
            rbD.move(rightSpd);

            delay(optimalDelay);
        }
    }

    void driveController()
    {
        while (!competition::is_disabled())
        {
            // std::cout << "drive\n";
            double leftSpd;
            double rightSpd;
            double leftSpdRaw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            double rightSpdRaw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

            leftSpd = leftSpdRaw * 12000 / 127;
            rightSpd = rightSpdRaw * 12000 / 127;

            lfD.move_voltage(leftSpd);
            lbD.move_voltage(leftSpd);
            rfD.move_voltage(rightSpd);
            rbD.move_voltage(rightSpd);
            delay(optimalDelay);
        }
        std::cout << "ended drive\n";
    }

    // voltage controller for flywheel motors
    void flyController()
    {
        while (!competition::is_disabled())
        {
            // std::cout << "fly\n";
            static double IPIDang = 0;
            static double IPIDang2 = 0;
            if (competition::is_disabled())
            {
                IPIDang = 0;
                IPIDang2 = 0;
            }
            double flyWVolt;
            double flyWVolt2;
            double flyWheelW = flyWheel1.get_actual_velocity();
            double flyWheelW2 = flyWheel2.get_actual_velocity();
            double diffFlyWheelW = angularVelocityCalc() - flyWheelW;
            double diffFlyWheelW2 = angularVelocityCalc() - flyWheelW2;
            static double prevFWdiffSPD = angularVelocityCalc();
            static double prevFWdiffSPD2 = angularVelocityCalc();

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

            if (flyWVolt > 12000)
            {
                flyWVolt = 12000;
            }
            if (flyWVolt < -12000)
            {
                flyWVolt = -12000;
            }
            if (fabs(diffFlyWheelW) < 1 && flyWVolt == 0)
            {
                IPIDang = 0;
            }

            if (flyWVolt2 > 12000)
            {
                flyWVolt2 = 12000;
            }
            if (flyWVolt2 < -12000)
            {
                flyWVolt2 = -12000;
            }
            if (fabs(diffFlyWheelW2) < 1 && flyWVolt2 == 0)
            {
                IPIDang2 = 0;
            }

            if (isnanf(flyWVolt))
            {
                flyWVolt = 0;
                prevFWdiffSPD = angularVelocityCalc();
                IPIDang = 0;
            }
            if (isnanf(flyWVolt2))
            {
                flyWVolt2 = 0;
                prevFWdiffSPD2 = angularVelocityCalc();
                IPIDang2 = 0;
            }

            flyWheel1.move_voltage(flyWVolt);
            flyWheel2.move_voltage(flyWVolt2);

            delay(optimalDelay);
        }
        std::cout << "ended fly\n";
    }

    // power controller for differential motors between intake and turret
    void turretIntakeController()
    {
        while (!competition::is_disabled())
        {

            turretMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
            if (!competition::is_autonomous())
            {
                ejectPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
                if (master.get_digital(E_CONTROLLER_DIGITAL_A) && master.get_digital(E_CONTROLLER_DIGITAL_X) &&
                    master.get_digital(E_CONTROLLER_DIGITAL_B) && master.get_digital(E_CONTROLLER_DIGITAL_Y))
                {
                    boomShackalacka.set_value(true);
                    shoot3.set_value(false);
                }
                else
                {
                    boomShackalacka.set_value(false);
                    shoot1.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
                    shoot3.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
                    ejectPiston.set_value(master.get_digital(pros::E_CONTROLLER_DIGITAL_X));
                }
            }

            double intakeSpd = intakeControl();
            double turrSpd = -turrControl()*12000/127;
            if (turrSpd == 0)
            {
                turretMotor.brake();
            }
            else
            {
                turretMotor.move_voltage(turrSpd);
            }

            intakeMotor.move(intakeSpd);

            delay(optimalDelay);
        }
        std::cout << "ended turret\n";
    }

    void raiseAScore(int times)
    {
        shoot3.set_value(true);
        delay(500);
        shoot3.set_value(false);
        if (times > 1)
        {
            intakeMotor.move(-127);
            delay(3000);
            intakeMotor.move(0);
            shoot3.set_value(true);
            delay(500);
            shoot3.set_value(false);
        }
    }

    void speedToggle(void)
    {
        static bool highspd = false;
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y))
        {
            highspd = !highspd;
        }
        if (highspd)
        {

            sensing.goalSpeed = 420;
        }
        else
        {

            sensing.goalSpeed = 312;
        }
    }
    void driveToRoller(void)
    {
        lfD.move_voltage(4000);
        lbD.move_voltage(4000);
        rfD.move_voltage(4000);
        rbD.move_voltage(4000);

        intakeMotor.move_voltage(12000);
        delay(500);
        intakeMotor.move_voltage(000);
        /*
        delay(500);
        spinRoller = true;
        int startTime = millis();
        while(spinRoller == true && millis() - startTime < 4000){
            static int pwr = 4000;
            if (!sensing.rollerIsGood() || fabs(intakeMotor.get_actual_velocity()-60) < 20){
                if (sensing.underRoller()){
                    if (fabs(intakeMotor.get_actual_velocity()) < 60){
                        if (pwr < 12000){
                            pwr+=50;
                        }
                        else{
                            pwr=12000;
                        }
                    }
                    else{
                        if (pwr > 2000){
                            pwr-=10;
                        }
                        else{
                            pwr = 2000;
                        }
                    }
                    intakeMotor.move_voltage(pwr);
                }
                else{
                    intakeMotor.move_voltage(12000);
                }

            }
            else{
                spinRoller = false;
            }
            delay(20);

        }
        */
    }

    void explode(void)
    {

        boomShackalacka.set_value(true);
    }
};

extern void drive_ControllerWrapper(void *mControl);
extern void turretIntake_ControllerWrapper(void *mControl);
extern void fly_ControllerWrapper(void *mControl);

#endif