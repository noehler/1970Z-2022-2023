#include "main.h"

using namespace pros;
sensing_t sensing;

double targetAngleOffest = 0;
double chaIntAng = 0;
double goalAngle = 0;

void odometry_Wrapper(void* sensing) {
    ((sensing_t*) sensing)->odometry();
}

void SSOSTTT_Wrapper(void* sensing){
    ((sensing_t*) sensing)->SSOSTTT();
}

void VT_Wrapper(void* sensing){
    ((sensing_t*) sensing)->visionTracking();
}
void GPS_Wrapper(void* sensing){
    ((sensing_t*) sensing)->GPS_tracking();
}
void inertial_tracking_Wrapper(void* sensing){
    ((sensing_t*) sensing)->inertial_tracking();
}

double getNum(std::string Output){
  std::string tempDist;
  double realNum;
  while (1){
    std::cout << "\n" << Output;
    std::cin >> tempDist;

    bool notValid = false;
    try{
      realNum = std::stod(tempDist);
    }
    catch(std::invalid_argument){
      notValid = true;
    }
    if (!notValid){
      break;
    }
    else{
      std::cout << "\n\n Please input valid number";
    }
  }
  return realNum;
}

Controller master(pros::E_CONTROLLER_MASTER);
Controller sidecar(pros::E_CONTROLLER_PARTNER);