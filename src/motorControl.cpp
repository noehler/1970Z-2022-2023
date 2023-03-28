#include "main.h"

//these are needed in order to activate tasks of members of a class
void drive_ControllerWrapper(void* mControl) {
    if (!competition::is_autonomous()){
        ((motorControl_t*) mControl)->driveController();
    }
    else{
        ((motorControl_t*) mControl)->autonDriveController();
    }
}
void intake_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->intakeController();
}
void fly_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->flyController();
}
 