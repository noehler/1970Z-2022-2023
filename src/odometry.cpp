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
    if (encoderLoc == &leftEncoderFB){
        radius=1.411811948;
    }
    if (encoderLoc == &rightEncoderFB){
        radius=1.449;
    }
    else{
        radius = 1.41326;
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
  //Arc1 = getNum("Arc1: ");
  double Arc2 =distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positive
  //Arc2 = getNum("Arc2: ");
  double Arc3 = distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
  //Arc3 = getNum("Arc3: ");

  int i = 0;
  
  double a = 8; //distance between two tracking wheels
  double b = -2.5; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
  static float T = 0;
  static double previousT =0;
  T = float(millis())/1000 - previousT; // JLO - Is this right?  What units are T in?  usec or sec?
  previousT+=T;
  double P1 = (Arc1 - Arc2);
  double Delta_y, Delta_x;
  double radRotation = -((inertial.get_rotation() * M_PI) / 180); 
  //radRotation = getNum("Angle: ");

  if (radRotation == PROS_ERR_F)
  {
    // JLO - handle error and exit, we can't continue
    return;
  }

  // relying on heading calibrated by odometry in order to reduce noise but also comparing it to inertial to check for drift
  if (fabs(odoHeading - radRotation) >= 0.1){
    odoHeading = radRotation;
    std::cout << "\n angleDiff too big";
  }
  //odoHeading = getNum("Heading: ");

  double Delta_heading = P1 / a; // change of heading

  //checkingVals
  /*logVals("Arc1", Arc1);
  logVals("Arc2" , Arc2);
  logVals("Arc3" , Arc3);
  logVals("P1" , P1);
  logVals("Angle" , radRotation);
  logVals("Time" , T);
  logVals("Delta Heading" , Delta_heading);*/

  if ( P1 != 0) { // if there are change of heading while moving, arc approximation
    double Radius_side = (Arc1 + Arc2)*a/(2*P1); // radius to either side of the robot
    double Radius_back = Arc3/Delta_heading - b; // radius to back or forward of the robot

    // Radius_back could be changed to cos(odoHeading + Delta_heading-M_PI/2) - cos(odoHeading - M_PI/2);
    // if are using encoder-based angle tracking ( recommanded for less noice)

    double cos_side = -sin(radRotation) + sin(radRotation-Delta_heading);
    double cos_back = -cos(radRotation) + cos(radRotation-Delta_heading);
    double sin_side = -cos(radRotation) + cos(radRotation-Delta_heading);
    double sin_back = -sin(radRotation) + sin(radRotation-Delta_heading);   

    Delta_x = -Radius_side * cos_side - Radius_back * cos_back;
    Delta_y = Radius_side * sin_side - Radius_back * sin_back;

    //outPutting vals
    /*logVals("Side Radius" , Radius_side);
    logVals("Back Radius" , Radius_back);
    logVals("cos side" , cos_side);
    logVals("cos back" , cos_back);
    logVals("sin side" , sin_side);
    logVals("sin back" , sin_back);
    logVals("deltaX" , Delta_x);
    logVals("deltaY" , Delta_y);*/
  } 
  else { // if there are no change of heading while moving, triangular approximation
    //std::cout << "\nNo diff in a1 and a2";
    Delta_x = Arc1 * cos(odoHeading) + (Arc3 * cos(odoHeading+(M_PI/2)));
    Delta_y = Arc1 * sin(odoHeading) + (Arc3 * sin(odoHeading+(M_PI/2)));
  }
  //std::cout << "\n DX: " << Delta_x << ", DY: " << Delta_y;
  odoHeading += Delta_heading;
  robot.xpos += Delta_x;
  robot.ypos += Delta_y;
  robot.xVelocity = Delta_x/T; // I need Change of time(time elapsed of each loop)
  robot.yVelocity = Delta_y/T; //same as above
  robot.wVelocity = Delta_heading/T;
  //outputting values
  /*logVals("xPos" , robot.xpos);
  logVals("yPos" , robot.ypos);
  logVals("xVel" , robot.xVelocity);
  logVals("yVel" , robot.yVelocity);
  logVals("reset");*/
}