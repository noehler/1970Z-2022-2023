#include "main.h"
#include "pros/misc.hpp"

//these are needed in order to activate tasks of members of a class
void drive_ControllerWrapper(void* mControl) {
    if (competition::is_autonomous()){
        ((motorControl_t*) mControl)->driveController();
    }
    else{
        ((motorControl_t*) mControl)->autonDriveController();
    }
}
void turretIntake_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->turretIntakeController();
}
void fly_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->flyController();
}
