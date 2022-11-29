#include "main.h"

using namespace pros;
Controller master(pros::E_CONTROLLER_MASTER);
Controller sidecar(pros::E_CONTROLLER_PARTNER);

double targetAngleOffest = 0;
double chaIntAng = 0;

void odometry_Wrapper(void* sensing) {
    ((sensing_t*) sensing)->odometry();
}

void SSOSTTT_Wrapper(void* sensing){
    ((sensing_t*) sensing)->SSOSTTT();
}