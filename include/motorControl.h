#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "pros/misc.hpp"
#ifndef _PROS_MAIN_H_
#include "api.h"
#endif

#ifndef __ROBOTCONFIG_H__
#include "robotConfig.h"
#endif

using namespace pros;

class motorControl_t{
    private:
        Motor lfD;
        Motor lbD;
        Motor rfD;
        Motor rbD; 
        Motor flyWheel1;
        Motor flyWheel2;
        Motor diff1;
        Motor diff2;
        ADIDigitalOut boomShackalacka;
        ADIDigitalOut shootPiston;
        ADIDigitalOut intakeLiftPiston;
        Controller master;
        Controller sidecar;
        int optimalDelay = 20;

        class PID_t{
            public:
                double p, i, d, p2, i2,d2;
        };
        class tunedSystems_t{
            public:
                PID_t driveFR, driveSS, turret, flyWheel;
        } PID;

        class moveToInfoInternal_t{
        public:
            double moveToxpos, moveToypos, targetHeading, ets, speed_limit=100, errtheta=5;
            double dist = 0;          // change of position
            double distR = 0;         // chagne of right postion
            double distL = 0;         // change of left position
            double PIDSS = 0;         // PID turning speed
            double PIDFW = 0;         // PID moveforward speed
            double PIDSpeedL = 0;     // PID leftside speed
            double PIDSpeedR = 0;     // PID rightside speed
            double prevPIDFW = 0;     // PID moveforward speed at t = -1
            double prevPIDSS = 0;     // PID turning speed at t = -1
            double PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
            double PIDSSFLAT = 0;   
            int moveToforwardToggle = 1, Stop_type = 2;
            double tolerance=5;
        };

        class moveToInfoExternal_t{
        public:
            double moveToxpos, moveToypos, targetHeading, speed_limit=100, errtheta=5;
            int moveToforwardToggle = 1, Stop_type = 2;
            double tolerance=5;
            bool resetMoveTo;
        };

        void moveTo(double &leftSpd, double &rightSpd){
            static moveToInfoInternal_t moveI;
            
            static double IPIDSS = 0;
            static double previousets = 0;
            static double IPIDfw = 0;
            static double previouset = 0;
            if (move.resetMoveTo) {
                IPIDSS = 0;
                previousets = 0;
                IPIDfw = 0;
                previouset = 0;
                moveI.dist = 0;          // change of position
                moveI.distR = 0;         // chagne of right postion
                moveI.distL = 0;         // change of left position
                moveI.PIDSS = 0;         // PID turning speed
                moveI.PIDFW = 0;         // PID moveforward speed
                moveI.PIDSpeedL = 0;     // PID leftside speed
                moveI.PIDSpeedR = 0;     // PID rightside speed
                moveI.prevPIDFW = 0;     // PID moveforward speed at t = -1
                moveI.prevPIDSS = 0;     // PID turning speed at t = -1
                moveI.PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
                moveI.PIDSSFLAT = 0;     // variable used for keeping turning speed < 20
                // variable for for calculating first turning.
                move.resetMoveTo = false;
            }
            double currentheading = sensing.robot.angle/180*M_PI;
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
            double etx = move.moveToxpos - sensing.robot.xpos;//change of x
            double ety = move.moveToypos - sensing.robot.ypos;//change of y
            double dist = sqrt(pow(etx, 2) + pow(ety, 2));
            double et = dist * 10;

            //std::cout << "\n Hi5";
            move.targetHeading = atan(ety/etx);
            if (etx<0){
                move.targetHeading +=M_PI;
            } else if(etx ==0){
                move.targetHeading = M_PI/2*(fabs(ety)/ety);
            }
            if (move.moveToforwardToggle == -1){
                move.targetHeading +=M_PI;
            }
            moveI.ets = move.targetHeading - currentheading;
            if (moveI.ets < -M_PI) {
            moveI.ets += 2*M_PI;
            }
            if (moveI.ets > M_PI) {
            moveI.ets -= 2*M_PI;
            }
            
            moveI.ets = moveI.ets*180/M_PI;
            IPIDSS += moveI.ets;
            IPIDfw += et;
            if (move.moveToforwardToggle == 1){
                moveI.PIDSS = 3 * moveI.ets + 0.1 * IPIDSS * .01 + 0.3 * (moveI.ets - previousets);
            }
            else{
                moveI.PIDSS = 3 * moveI.ets + 0.1 * IPIDSS * .01 + 0.3 * (moveI.ets - previousets);
            }
            if (fabs(moveI.ets) < 10) {
                if (move.moveToforwardToggle == 1){
                moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }
                else{
                moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
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
            if (dist < move.tolerance) {
                move.resetMoveTo = true;
                if (move.Stop_type == 1) {
                    //motor stop (hold)
                    leftSpd = 0;
                    rightSpd = 0;
                    lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                    rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
                } else {
                    //motor stop (coast)
                    leftSpd = 0;
                    rightSpd = 0;
                    lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                    rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);

                }
            } else {
            
                leftSpd = moveI.PIDSpeedL;
                rightSpd = moveI.PIDSpeedR;
            }
        }

        double angularVelocityCalc(void){
            return 1;
        }

        bool recoilPrevent;
        int intakeRunning;
        double turrControl(void){
            static double PIDPosition = 0;
            static double PIDVelocity = 0;
            static double T = 0;
            static double previousT=0;  
            static double PIDscalar = 1.5;
            static double gyroScalar = 21.5833333;
            static double chassisScalar = 21.5833333;
            static double turPredicScalar = 21.5833333;
            static double angdiff;
            T = float(millis())/1000 - previousT;
            previousT+=T;
            /*if (!competition::is_disabled() && !competition::is_autonomous()){
                robotGoal.angleBetweenHorABS = robot.angle + 180;
            }*/
            angdiff = angleBetween - sensing.robot.turAng;
            if (angdiff > 180){
                angdiff -= 360;
            }
            else if( angdiff < -180){
                angdiff += 360;
            }
            if (fabs(angdiff) < 2){
                angdiff = 0;
            }
            std::cout << "\n" << angdiff;


            static double IPIDvel = 0;
            static double previousveldiff = 0;
            static double IPIDang = 0;
            static double previousangdiff = 0;
            if (!competition::is_disabled()){
                IPIDang += angdiff;
                PIDPosition =(PID.turret.p*angdiff + PID.turret.i*IPIDang + PID.turret.d*(angdiff - previousangdiff));
                previousangdiff = angdiff;
                double veldiff = gyroScalar*T*(sensing.robot.angAccel)-sensing.robot.wVelocity*chassisScalar + turPredicScalar*sensing.robot.turvelocity+PIDPosition*PIDscalar + 0.025*recoilPrevent*sensing.goalSpeed;
                IPIDvel += veldiff;
                PIDVelocity =(0.415*veldiff + 0.135*IPIDvel*.01 + 2.6*(veldiff - previousveldiff));
                previousveldiff = veldiff;
                if (fabs(angdiff) == 0 || PIDPosition==0){
                IPIDang = 0;
                }
                if (fabs(veldiff)<0.1){
                IPIDvel = 0;
                }
            }
            else{
                PIDVelocity = 0;
            }
            
            
            if (sensing.underRoller){
                PIDVelocity = 0;
            }
            if (intakeRunning != 0){
                PIDVelocity = 0;
                IPIDvel = 0;
                IPIDang = 0;
            }

            return PIDVelocity;
        }

        double intakeControl(double diffInSpd){
            int baseSPD;
            if (intakeRunning == 2){
                baseSPD = 127-fabs(diffInSpd);
            }
            else if (intakeRunning == 1){
                baseSPD = -127+fabs(diffInSpd);
            }
            else{
                baseSPD = 0;
            }

            if (sensing.underRoller){
                baseSPD *= .5;
            }
            return baseSPD;
        }
    public:
        //Constructor to assign values to the motors and PID values
        motorControl_t(void): lfD(1, E_MOTOR_GEARSET_06, true), lbD (2, E_MOTOR_GEARSET_06, true), rfD(3, E_MOTOR_GEARSET_06, false), rbD(4, E_MOTOR_GEARSET_06, false), 
                                                    flyWheel1(7, E_MOTOR_GEARSET_06, false), flyWheel2(8, E_MOTOR_GEARSET_06, true),
                                                    diff1(5, E_MOTOR_GEARSET_06, true), diff2(6, E_MOTOR_GEARSET_06, true), boomShackalacka({{22,'C'}}), shootPiston({{22,'B'}}), intakeLiftPiston({{9,'G'}}),
                                                    master(pros::E_CONTROLLER_MASTER), sidecar(pros::E_CONTROLLER_PARTNER){
            PID.driveFR.p = 1;
            PID.driveFR.i = 0;
            PID.driveFR.d = 3;

            PID.turret.p = 1.5;
            PID.turret.i = .167;
            PID.turret.d = -.66492;

            PID.turret.p2 = 0.415;
            PID.turret.i2 = 0.00135;
            PID.turret.d2 = 2.6;

            PID.flyWheel.p = .20549;
            PID.flyWheel.i = 0.0215;
            PID.flyWheel.d = .10409;
            PID.flyWheel.p2 = .0;
        }
        moveToInfoExternal_t move;
        
        void autonDriveController(void){
            while(!competition::is_disabled()){
                
                static double leftSpd = 0;
                static double rightSpd = 0;
                static int diffDir = 1;
                
                moveTo(leftSpd, rightSpd);

                if (diffDir == 1 && fabs(lfD.get_actual_velocity()) < 130){
                    diffDir = -1;
                }

                if (diffDir == -1 && fabs(lfD.get_actual_velocity()) > 190){
                    diffDir = 1;
                }
                lfD.move(leftSpd * diffDir);
                lbD.move(leftSpd);
                rfD.move(rightSpd * diffDir);
                rbD.move(rightSpd);

                delay(optimalDelay);
            }
        }

        void driveController(){
            while(!competition::is_disabled()){
                //std::cout << "drive\n";
                static double leftSpd = 0;
                static double rightSpd = 0;
                if (competition::is_autonomous()){

                } else{ // usercontrol loop
                    leftSpd = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		            rightSpd = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                }
                static int diffDir = 1;
                
                if (diffDir == 1 && fabs(lfD.get_actual_velocity()) < 130){
                    diffDir = -1;
                }

                if (diffDir == -1 && fabs(lfD.get_actual_velocity()) > 190){
                    diffDir = 1;
                }

                lfD.move(leftSpd * diffDir);
                lbD.move(leftSpd);
                rfD.move(rightSpd * diffDir);
                rbD.move(rightSpd);

                delay(optimalDelay);
            }
            std::cout << "ended drive\n";
        }
        
        //voltage controller for flywheel motors
        void flyController(){
            while(!competition::is_disabled()){
                //std::cout << "fly\n";
                static double IPIDang = 0;
                if (competition::is_disabled()){
                    IPIDang = 0;
                }
                double flyWVolt;
                double flyWheelW =(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity())/2;
                double diffFlyWheelW = angularVelocityCalc()-flyWheelW;
                static double prevFWdiffSPD = angularVelocityCalc();

                IPIDang += diffFlyWheelW;
                double prop = PID.flyWheel.p*diffFlyWheelW;
                double integ = IPIDang*PID.flyWheel.i;
                double deriv = PID.flyWheel.d*(diffFlyWheelW - prevFWdiffSPD);
                double prop2 = PID.flyWheel.p2 * flyWheel1.get_actual_velocity();
                prevFWdiffSPD = diffFlyWheelW;
                flyWVolt = 12000.0/127*(prop + integ + deriv + prop2);
                if (flyWVolt > 12000){
                    flyWVolt = 12000;
                }
                if (flyWVolt < -12000){
                    flyWVolt = -12000;
                } 
                if (fabs(diffFlyWheelW)<1&&flyWVolt==0){
                    IPIDang = 0;
                }

                flyWheel1.move_voltage(flyWVolt); 
                flyWheel2.move_voltage(flyWVolt); 

                delay(optimalDelay);
            }
            std::cout << "ended fly\n";
        }
        
        //power controller for differential motors between intake and turret
        void turretIntakeController(){
            while(!competition::is_disabled()){
                double diffInSpd = turrControl();
                double baseSpd = intakeControl(diffInSpd);
                diff1 = diffInSpd + baseSpd;
                diff2 = -diffInSpd + baseSpd;
                delay(optimalDelay);
            }
            std::cout << "ended turret\n";
        }
};

extern void drive_ControllerWrapper(void* mControl);
extern void turretIntake_ControllerWrapper(void* mControl);
extern void fly_ControllerWrapper(void* mControl);

#endif