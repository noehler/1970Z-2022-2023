#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "pros/misc.hpp"
#ifndef _PROS_MAIN_H_
#include "api.h"
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

        double angularVelocityCalc(void){
            return 1;
        }
    public:
        //Constructor to assign values to the motors and PID values
        motorControl_t(void): lfD(1, E_MOTOR_GEARSET_06, true), lbD (2, E_MOTOR_GEARSET_06, true), rfD(7, E_MOTOR_GEARSET_06, false), rbD(4, E_MOTOR_GEARSET_06, false), 
                                                    flyWheel1(11, E_MOTOR_GEARSET_06, false), flyWheel2(13, E_MOTOR_GEARSET_06, true),
                                                    diff1(5, E_MOTOR_GEARSET_06, true), diff2(6, E_MOTOR_GEARSET_06, true), boomShackalacka({{22,'C'}}), shootPiston({{22,'B'}}), intakeLiftPiston({{9,'G'}}),
                                                    master(pros::E_CONTROLLER_MASTER), sidecar(pros::E_CONTROLLER_PARTNER){PID.driveFR.p = 1;
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
        
        void driveController(){
            while(!competition::is_disabled()){
                std::cout << "drive\n";
                static double leftSpd = 0;
                static double rightSpd = 0;
                if (competition::is_autonomous()){

                } else{ // usercontrol loop
                    double leftSpd = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		            double rightSpd = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
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
                std::cout << "fly\n";
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

                std::cout << "turret\n";
                delay(optimalDelay);
            }
            std::cout << "ended turret\n";
        }
};

extern void drive_ControllerWrapper(void* mControl);
extern void turretIntake_ControllerWrapper(void* mControl);
extern void fly_ControllerWrapper(void* mControl);

#endif