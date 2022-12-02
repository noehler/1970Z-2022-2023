#include "main.h"

//these are needed in order to activate tasks of members of a class
void drive_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->driveController();
}
void turretIntake_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->turretIntakeController();
}
void fly_ControllerWrapper(void* mControl) {
    ((motorControl_t*) mControl)->flyController();
}
