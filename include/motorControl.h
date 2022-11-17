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

public:
    //Constructor to assign values to the motors
    motorControl_t(void): lfD(1, E_MOTOR_GEARSET_06, true), lbD (2, E_MOTOR_GEARSET_06, true), rfD(7, E_MOTOR_GEARSET_06, false), rbD(4, E_MOTOR_GEARSET_06, false), 
                                                flyWheel1(11, E_MOTOR_GEARSET_06, false), flyWheel2(13, E_MOTOR_GEARSET_06, true),
                                                diff1(5, E_MOTOR_GEARSET_06, true), diff2(6, E_MOTOR_GEARSET_06, true){
    }
    void driveController(void){
        while(!competition::is_disabled()){

        }
    }
    void turretIntakeController(void){
        while(!competition::is_disabled()){

        }
    }
    void flyController(void){
        while(!competition::is_disabled()){

        }
    }
};

#endif