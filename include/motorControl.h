#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

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
        ADIDigitalOut elevatePiston;

    public:
        //Constructor to assign values to the motors
        motorControl_t(void): lfD(1, E_MOTOR_GEARSET_06, true), lbD (2, E_MOTOR_GEARSET_06, true), rfD(7, E_MOTOR_GEARSET_06, false), rbD(4, E_MOTOR_GEARSET_06, false), 
                                                    flyWheel1(11, E_MOTOR_GEARSET_06, false), flyWheel2(13, E_MOTOR_GEARSET_06, true),
                                                    diff1(5, E_MOTOR_GEARSET_06, true), diff2(6, E_MOTOR_GEARSET_06, true), boomShackalacka({{22,'C'}}), shootPiston({{22,'B'}}), elevatePiston({{9,'G'}}){
        }
        void driveController(){
            while(!competition::is_disabled()){

            }
        }
        void turretIntakeController(){
            while(!competition::is_disabled()){

            }
        }
        void flyController(){
            while(!competition::is_disabled()){

            }
        }
};

extern void drive_ControllerWrapper(void* mControl);
extern void turretIntake_ControllerWrapper(void* mControl);
extern void fly_ControllerWrapper(void* mControl);

#endif