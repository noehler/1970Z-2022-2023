// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"
#include "pros/vision.h"
#include "robotConfig.h"
#include <cmath>
#define DEG2RAD M_PI/180

position_t robot;
position_t homeGoal;
robotGoalRelatives robotGoal;
double magnitude(double a,double b){
  return sqrt(pow(a,2)+pow(b,2));
}
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

double odoHeading = robot.chaIntAng*M_PI/180;
double radRotation = 0;

void odometry(void){ // encoder odom, primary odom,
  static double odomposx = 0;
  static double odomposy = 0;
  double Arc1 =distTraveled(&rightEncoderFB); //rightEncoderFB travel, to forward direction of robot is positive
  double Arc2 =distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positiv
  double Arc3 = distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
  double a = 8; //distance between two tracking wheels
  double b = -2.5; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
  double P1 = (Arc1 - Arc2);
  double Delta_y, Delta_x;
  double radRotation = mod(2*M_PI,(-inertial.get_heading()+robot.chaIntAng)*M_PI/180);
  if (radRotation == PROS_ERR_F)
  {
    // JLO - handle error and exit, we can't continue
    std::cout<<"chassis inertia malfunction";
    return;
  }

  double angle_error = odoHeading - radRotation;
  if (angle_error > M_PI){
    angle_error -=2*M_PI;
  } else if (angle_error<-M_PI){
    angle_error +=2*M_PI;
  }
  // relying on heading calibrated by odometry in order to reduce noise but also comparing it to inertial to check for drift
  if (fabs(angle_error) >= 0.1){
    odoHeading = radRotation;
    std::cout << "\n chassis heading error"<<angle_error;
  }

  double Delta_heading = P1 / a; // change of heading


  if ( P1 != 0) { // if there are change of heading while moving, arc approximation
    double Radius_side = (Arc1 + Arc2)*a/(2*P1); // radius to either side of the robot
    double Radius_back = Arc3/Delta_heading - b; // radius to back or forward of the robot
    double cos_side = sin(odoHeading+Delta_heading) - sin(odoHeading);
    double cos_back = -cos(odoHeading+Delta_heading) + cos(odoHeading);
    double sin_side = -cos(odoHeading+Delta_heading) + cos(odoHeading);
    double sin_back = -sin(odoHeading+Delta_heading) + sin(odoHeading);

    Delta_x = Radius_side * cos_side - Radius_back * cos_back;
    Delta_y = Radius_side * sin_side - Radius_back * sin_back;

  }
  else {
    Delta_x = Arc1 * cos(odoHeading) - (Arc3 * cos(odoHeading+(M_PI/2)));
    Delta_y = Arc1 * sin(odoHeading) - (Arc3 * sin(odoHeading+(M_PI/2)));
  }
  odoHeading += Delta_heading;
  odoHeading = mod(2*M_PI,odoHeading);
  robot.angle = odoHeading*180/M_PI;
  static float T = 0;
  static double previousT =0;
  T = float(millis())/1000 - previousT;
  previousT+=T;
  robot.wVelocity = Delta_heading/T;
  robot.xVelocity = Delta_x/T; 
  robot.yVelocity = Delta_y/T; 
  robot.xposodom += Delta_x;
  robot.yposodom += Delta_y;
  if (fabs(Delta_y) > 10 && usd::is_installed()){
    outValsSDCard();
  }
}

void turAngupdate(){
  static double error =0;
  double encAng = double(turretEncoder.get_position())/2158.3333- error;//convert to angle btw turret and robot chassie
  double turrHeadingEnc = mod(360,robot.TurintAng-robot.chaIntAng+encAng+robot.angle);//
  double turrHeadingInr = mod(360,-inertialTurret.get_heading()+robot.TurintAng);
  
  double angle_error = turrHeadingEnc- turrHeadingInr;
  if (angle_error > 180){
    angle_error -=360;
  } else if (angle_error<-180){
    angle_error +=360;
  }
  // relying on heading calibrated by odometry in order to reduce noise but also comparing it to inertial to check for drift
  if (fabs(angle_error) >= 2){
  error += angle_error;
    //std::cout << "\n turret angle error"<<angle_error;
  }
  robot.turAng = turrHeadingEnc + targetAngleOffest;
}

void visionOdom(){  
  //double camera obejct localization
  //need to know object height, distance between camera's optical axis
  //also convert to global coordinate according to some offsets
  //conversion from x position to angular positions
  double angL = 1;//targetxL*0.189873418; //targetxL horizontal position of target center from left camera
  double angR = 1;//targetxR*0.189873418; //targetxR horizontal position of target center from right camera
  double A = 7.5625; //idstance from left camera optical axis to right camera optical axis
  //calculations of target postion in local coordinate
  double localY = (tan(angL)*A)/(tan(angL)+tan(angR)) - A/2;
  double localX = (localY+A/2)*tan(angL);
  //merging with local offsets
  double offsetLocalx = 3;
  double offsetLocaly = 0;
  double r = magnitude((offsetLocalx+localX),(offsetLocaly+localY));
  double tarDirection = atan((offsetLocaly+localY)/(offsetLocalx+localX));
  //convsersion to global coordinate
  //converting local displacement to global scope
  double Xc = r*cos(tarDirection + robot.turAng);
  double Yc = r*sin(tarDirection + robot.turAng);
  //calculating robot position
  robot.xposvision = homeGoal.xpos - Xc;
  robot.yposvision = homeGoal.ypos - Yc;
} 
void singleEyeVision(){
  double theta = 1;//object center horizontal angle with respect to optical axis, positive right
  double phi = 1;//object center vertical angle with respec to optical axis, positive up
  double h = 14.2;//camera sensor height
  double targetH = 20;
  double phiOffset = 33.7;//camera installed tilte, deg
  //solve for ditance to target
  double localDistToTar = (targetH-h)/sin(phi+phiOffset);
  //solve for local displacements to target
  double localX = localDistToTar * cos(phi+phiOffset) * cos(theta);
  double localY = localDistToTar * cos(phi+phiOffset) * sin(theta);
  //merging with local offsets
  double offsetLocalx = 3;
  double offsetLocaly = 0;
  double r = magnitude((offsetLocalx+localX),(offsetLocaly+localY));
  double tarDirection = atan((offsetLocaly+localY)/(offsetLocalx+localX));
  //convsersion to global coordinate
  //converting local displacement to global scope
  double Xc = r*cos(tarDirection + robot.turAng);
  double Yc = r*sin(tarDirection + robot.turAng);
  //calculating robot position
  robot.xposvision = homeGoal.xpos - Xc;
  robot.yposvision = homeGoal.ypos - Yc;
}