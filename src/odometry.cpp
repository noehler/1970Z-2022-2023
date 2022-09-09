// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"
#include "robotConfig.h"
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


    //transfering from memory location to value at that memory location
    ADIEncoder encoderVar = *encoderLoc;

    double degreesTraveled = encoderVar.get_value();
    if (resetEncoder == true){
        encoderVar.reset();
    }

    double distTraveled = degreesTraveled/360 * radius *2*M_PI;

    return distTraveled;
}
 
void basicOdometry(void){
    //getting average of fwd reverse distance traveled to account for turning
    double avgFR = (distTraveled(&leftEncoderFB) + distTraveled(&rightEncoderFB))/2;
    double avgSS = distTraveled(&encoderLR);

    //converting from relative to robot movement to absolute to field movement
    double changeX = avgFR * cos(inertial.get_heading()*DEG2RAD) + avgSS * cos((90-inertial.get_heading())*DEG2RAD);
    double changeZ = avgFR * sin(inertial.get_heading()*DEG2RAD) + avgSS * sin((90-inertial.get_heading())*DEG2RAD);
    //std::cout << "\n\nCX: " << changeX << "\nCZ: " << changeZ  << "\nCXFR: " << avgFR * cos(inertial.get_heading()*DEG2RAD) << "\nCXSS: " << avgSS * cos((90-inertial.get_heading())*DEG2RAD) << "\n";
    //getting the angle moving, because may not always be moving straight forward
    robot.angle = atan(changeZ/changeX)/DEG2RAD;

    //adding calculated values onto the global position of the robot
    robot.xpos += changeX;
    robot.zpos += changeZ;
}

double velocityCalc(void){
    static double prevAvg = double(leftEncoderFB.get_value() + rightEncoderFB.get_value())/2;
    static double prevSS = encoderLR.get_value();
    static double prevT = pros::millis();
    double c = sqrt(pow((prevAvg - (leftEncoderFB.get_value())* + rightEncoderFB.get_value())/2000 * (prevT - millis()), 2) +
                    pow((prevSS-encoderLR.get_value())/1000 * (prevT - millis()), 2));

    prevAvg = double(leftEncoderFB.get_value() + rightEncoderFB.get_value())/2;
    prevSS = encoderLR.get_value();
    prevT = millis();
    return c;
}

void odometry(void){
  double Arc1 = distTraveled(&rightEncoderFB); //rightEncoderFB travel, to forward direction of robot is positive
  double Arc2 = distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positive
  double Arc3 = distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
  float a = robot.width; //distance between two tracking wheels
  float b = robot.length/2; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
  static float T =0;
  T = float(millis())/1000 - T;

  double P1 = (Arc1 - Arc2);
  double Delta_heading = P1/a;
  double Radius_side = (Arc1 + Arc2)*a/(2*P1);
  double Radius_back = Arc3/Delta_heading - b;
  double C_theta_side = cos(robot.angle+Delta_heading)-cos(robot.angle);
  double C_theta_back = cos(robot.angle+Delta_heading-M_PI/2)-cos(robot.angle-M_PI/2);
  double Delta_x = C_theta_side*Radius_side + C_theta_back * Radius_back;
  double S_theta_side = sin(robot.angle+Delta_heading)-sin(robot.angle);
  double S_theta_back = sin(robot.angle+Delta_heading-M_PI/2)-sin(robot.angle-M_PI/2);
  double Delta_y = S_theta_side*Radius_side + S_theta_back * Radius_back;
  robot.xpos +=Delta_x;
  robot.ypos +=Delta_y;
  robot.xVelocity = Delta_x/T; // I need Change of time(time elapsed of each loop)
  robot.yVelocity = Delta_y/T; //same as above
}
