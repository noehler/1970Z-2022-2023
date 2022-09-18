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

double odoHeading = 0;
double radRotation = -M_PI/2;
void odometry(void){
  double Arc1 =distTraveled(&rightEncoderFB); //rightEncoderFB travel, to forward direction of robot is positive
  Arc1 = getNum("Arc1: ");
  double Arc2 =distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positive
  Arc2 = getNum("Arc2: ");
  double Arc3 = distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
  Arc3 = getNum("Arc3: ");
  float a = 8; //distance between two tracking wheels
  float b = 3.5; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
  static float T = 0;
  T = float(millis())/1000 - T; // JLO - Is this right?  What units are T in?  usec or sec?
  double P1 = (Arc1 - Arc2);
  double Delta_y, Delta_x;
  double radRotation = -((inertial.get_rotation() * M_PI) / 180); 
  if (radRotation == PROS_ERR_F)
  {
    // JLO - handle error and exit, we can't continue
    return;
  }

  // relying on heading calibrated by odometry in order to reduce noise but also comparing it to inertial to check for drift
  if (fabs(odoHeading - radRotation) >= 5){
    odoHeading = radRotation;
  }
  odoHeading = getNum("Heading: ");

  double Delta_heading = P1 / a; // change of heading
  if ( P1 != 0) { // if there are change of heading while moving, arc approximation
    double Radius_side = (Arc1 + Arc2)*a/(2*P1); // radius to either side of the robot
    double Radius_back = Arc3/Delta_heading - b; // radius to back or forward of the robot

    // Radius_back could be changed to cos(odoHeading + Delta_heading-M_PI/2) - cos(odoHeading - M_PI/2);
    // if are using encoder-based angle tracking ( recommanded for less noice)
    double cos_side = cos(odoHeading+ Delta_heading-M_PI/2) - cos(odoHeading -M_PI/2);
    double cos_back = cos(odoHeading+ Delta_heading-M_PI) - cos(odoHeading -M_PI);
    double sin_side = sin(odoHeading+ Delta_heading-M_PI/2) - sin(odoHeading -M_PI/2);
    double sin_back = sin(odoHeading+ Delta_heading-M_PI) - sin(odoHeading  -M_PI);   
    std::cout <<"\n cos_side" << cos_side <<", cos_back" << cos_back;
    std::cout <<"\n sin_side" << sin_side <<", sin_back" << sin_back;

    Delta_x = Radius_side * cos_side - Radius_back * cos_back;
    Delta_y = Radius_side * sin_side - Radius_back * sin_back;
    std::cout << "\n DX: " << Delta_x << ", DY: " << Delta_y;
  } 
  else { // if there are no change of heading while moving, triangular approximation
    Delta_x = Arc1 * cos(odoHeading) + (Arc3 * cos(odoHeading+(M_PI/2)));
    Delta_y = Arc1 * sin(odoHeading) + (Arc3 * sin(odoHeading+(M_PI/2)));
  }
  odoHeading += Delta_heading;
  robot.xpos += Delta_x;
  robot.ypos += Delta_y;
  robot.xVelocity = Delta_x/T; // I need Change of time(time elapsed of each loop)
  robot.yVelocity = Delta_y/T; //same as above
}