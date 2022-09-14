// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"
#include "robotConfig.h"
#include <cmath>
#define DEG2RAD M_PI/180

position_t robot;
position_t homeGoal;
robotGoalRelatives robotGoal;

//function to automatically detect distance covered by an individual tracking wheel
//using pointers so that I can determine what ratio is needed to convert from degrees to distance without using a second variable
double distTraveled(ADIEncoder * encoderLoc, bool resetEncoder = true){
    double radius;
    if ((encoderLoc == &leftEncoderFB) || (encoderLoc == &rightEncoderFB) || (encoderLoc == &encoderLR)){
        radius=1.40723013;
    }
    else{
        radius = 0;
    }

    double degreesTraveled = encoderLoc->get_value();
    
    if (resetEncoder == true){
        encoderLoc->reset();
    }

    double distTraveled = (degreesTraveled/360) * radius * 2 * M_PI;

    return distTraveled;
}

double radRotation = -M_PI/2;
void odometry(void){
  double Arc1 =distTraveled(&rightEncoderFB); //rightEncoderFB travel, to forward direction of robot is positive
  double Arc2 =distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positive
  double Arc3 = distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
  float a = 8; //distance between two tracking wheels
  float b = 3.5; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
  static float T = 0;
  T = float(millis())/1000 - T; // JLO - Is this right?  What units are T in?  usec or sec?
  double P1 = (Arc1 - Arc2);
  double Delta_y, Delta_x;
  double rotation = inertial.get_rotation();
  if (rotation == PROS_ERR_F)
  {
    // JLO - handle error and exit, we can't continue
    return;
  }
  double radRotation = -((rotation * M_PI) / 180) - (M_PI / 2);
  double Delta_heading = P1 / a;
  static double pDH = 0;
  static double pDHM = 0;
  if ( P1 != 0) {
    double Radius_side = ((Arc1 + Arc2) * a) / (2 * P1);
    double Radius_back = (Arc3 / Delta_heading) - b;
    /*
        Arc3*a
        
    */
    double C_theta_side = cos(radRotation + Delta_heading) - cos(radRotation);
    double C_theta_back = cos(radRotation + Delta_heading - (M_PI/2)) - cos(radRotation - (M_PI/2));
    Delta_x = (C_theta_side * Radius_side) + (C_theta_back * Radius_back);
    double S_theta_side = sin(radRotation + Delta_heading) - sin(radRotation);
    double S_theta_back = sin(radRotation + Delta_heading - (M_PI/2)) - sin(radRotation-(M_PI/2));
    Delta_y = (S_theta_side*Radius_side) + (S_theta_back * Radius_back);
  }
  else {
    Delta_x = Arc1 * cos(radRotation) + (Arc3 * cos(radRotation-(M_PI/2)));
    Delta_y = Arc1 * sin(radRotation) + (Arc3 * sin(radRotation-(M_PI/2)));
  }

  //radRotation+=Delta_heading;
  std::cout << "\nPrevInertialRO: " << pDH - pDHM << ", CalcPrevH: " << Delta_heading;
  pDHM = pDH;
  pDH = radRotation;
  robot.xpos += Delta_x;
  robot.ypos += Delta_y;
  robot.xVelocity = Delta_x/T; // I need Change of time(time elapsed of each loop)
  robot.yVelocity = Delta_y/T; //same as above
}