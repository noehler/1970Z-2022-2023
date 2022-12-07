#include "main.h"

using namespace pros;
sensing_t sensing;

double targetAngleOffest = 0;
double chaIntAng = 0;
double angleBetween;

void odometry_Wrapper(void* sensing) {
    ((sensing_t*) sensing)->odometry();
}

void SSOSTTT_Wrapper(void* sensing){
    ((sensing_t*) sensing)->SSOSTTT();
}