#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

#include "Autons/autonSetup.h"
#include "GUI.h"
#include "api.h"
#include "output.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"


using namespace pros;

extern Controller master;
extern Controller sidecar;

extern bool highTurretInitAng;

extern double getNum(std::string Output);

// object class mainly used for the robot, but also used for defining goal
// position
class Object {
public:
  double xpos, ypos, zpos, // seperate position outputs for each tracking system
      odoxpos, odoypos,    // seperate position outputs for each tracking system
      GPSxpos, GPSypos,    // seperate position outputs for each tracking system
      angle,odoangle, turAng, turvelw, velX, velY, velW, turvelocity, odovelW, imuvelw,
      angAccel, xAccel, yAccel;
  int magFullness;
};

extern double targetAngleOffest;
extern double chaIntAng;
extern double goalAngle;
class sensing_t {
private:
  // JOHN: When calibrating make sure that the id(first arguement) is 1
  vision_signature_s_t REDGOAL = Vision::signature_from_utility(
      1, 4499, 8193, 6346, -1589, -429, -1009, 2.5, 0);
  // JOHN: When calibrating make sure that the id(first arguement) is 2
  vision_signature_s_t BLUEGOAL = Vision::signature_from_utility(
      2, -2553, -1927, -2240, 1009, 3205, 2107, 2.4, 0);

  class robotGoalRelatives {
  public:
    // storing the absolute and relative horizontal angle between goal and robot
    double dx, dy, dz;
  } robotGoal;

  // converts from degrees traveled to distance traveled
  double distTraveled(ADIEncoder *encoderLoc, bool resetEncoder = true) {
    double radius;
    if (encoderLoc == &leftEncoderFB) {
      radius = 1.375;
    }
    if (encoderLoc == &rightEncoderFB) {
      radius = 1.375;
    } else {
      radius = 1.383;
    }

    double degreesTraveled = encoderLoc->get_value();

    if (resetEncoder == true) {
      encoderLoc->reset();
    }

    double distTraveled = (degreesTraveled / 360) * radius * 2 * M_PI;

    return distTraveled;
  }

  // macro for pythagorean theorum because used a lot
  double magnitude(double a, double b) { return sqrt(pow(a, 2) + pow(b, 2)); }

  // macro to place a variable between +- a value(like degrees +-180)
  double mod(double base, double var) {
    while (base < var) { // e.g. 361mod360 = 1
      var -= base;
    }
    while (var < 0) { // e.g. -361mod 360 = 359
      var += base;
    }
    return var;
  }

  int optimalDelay = 20;

public:
  // declaring all sensors
  ADIEncoder leftEncoderFB;
  ADIEncoder rightEncoderFB;
  ADIEncoder encoderLR;
  ADIPotentiometer potentiometer;
  Imu inertial2;

  Imu inertial;

  Optical leftOpticalSensor;
  Optical rightOpticalSensor;
  Vision discSearch;

  GPS GPS_sensor;

  Object robot;
  Object goal;
  double goalSpeed = 0;
  bool magFull;

  // called at the start of class, defines sensors
  sensing_t(void)
      : leftEncoderFB({{5, 'E', 'F'}, false}),
        rightEncoderFB({{5, 'C', 'D'}, false}), encoderLR({{5, 'A', 'B'}, false}),
        inertial2(20), inertial(19),
        leftOpticalSensor(12), rightOpticalSensor(14), discSearch(16), potentiometer({22, 'e'}),
        GPS_sensor(16) {}

  // called at start of pre Auton, calibrates inerials and other sensors along
  // with setting intial values
  void Init(void) {
    // calibrating inertial sensors
    static bool inertialsSet = false;
    if (!inertialsSet) {
      inertial.reset();
      inertial2.reset();
      int startTime = 0;
      while (inertial.is_calibrating() || inertial2.is_calibrating()) {
        if (millis() - startTime > 3000) {
          master.clear_line(2);
          delay(50);
          master.print(2, 0, "CalibrationFailing");
        }
        if (millis() - startTime > 3000 &&
            master.get_digital(E_CONTROLLER_DIGITAL_B)) {
          break;
        }
        delay(50);
        std::cout << "calibrating\n";
      }
      delay(1000);
      std::cout << "calibrated\n";
      inertialsSet = true;
    }

    // setting up values for sensors
    GPS_sensor.set_offset(0, 0);
    inertial.set_heading(0);
    inertial2.set_heading(0);

    master.print(2, 0, "Calibration Done");
  }

  // color = true > red. color = false > blue.
  //  function used to set up position and angle along with other variables at
  //  start of autonomous
  void set_status(double xpos, double ypos, double heading,
                  double color_sensor_luminance, int colorPT) {
    leftOpticalSensor.set_led_pwm(color_sensor_luminance);
    rightOpticalSensor.set_led_pwm(color_sensor_luminance);
    robot.xpos = xpos;
    robot.ypos = ypos;
    robot.odoxpos = xpos;
    robot.odoypos = ypos;
    robot.zpos = 8;
    chaIntAng = heading;
    robot.odoangle = chaIntAng;
    odoHeading = chaIntAng/180*M_PI;
    color = colorPT;
    goal.xpos = 20;
    goal.ypos = 124;
    goal.zpos = 30;
  }

  // used to get data form GPS sensor and mix it with odometry
  void GPS_tracking(void) {
    // note: rotation of Gps strip can vary depend on field if true, going to
    // verify on 2/28
    bool sensorFail = false;
    while (1) {
      static bool prevX;
      static bool prevY;
      static double ydiff = 0, xdiff = 0; // diff from middle/gps origin
      pros::c::gps_status_s_t temp_status = GPS_sensor.get_status();
      if (color != 0) {
        xdiff = double(temp_status.y) * 39.37;
        ydiff = -double(temp_status.x) * 39.37;
      } else {
        xdiff = -double(temp_status.y) * 39.37;
        ydiff = double(temp_status.x) * 39.37;
      }

      static bool firstFail = true;
      // if red: x direction is correct y is flipped, if blue: x direction is
      // flipped, y is correct
      if (prevX - (72 + xdiff) == 0) {
        sensorFail = true;

        if (firstFail) {
          master.clear_line(2);
          delay(50);
          master.print(2, 0, "GPS Fail");
          firstFail = false;
        }
      } else {
        sensorFail = false;
      }
      prevX = (72 + xdiff);
      robot.GPSxpos = 72 + xdiff - cos(robot.turAng * M_PI / 180) * 4.5;
      robot.GPSypos = 72 + ydiff - sin(robot.turAng * M_PI / 180) * 4.5;

      delay(20);
    }
  }

  //odometry task used to track position with tracking wheels placed under robot
  double arc1g;
  double arc2g;
  double arc3g;
  double odoHeading = 0;
  void odometry(void) {

    while (1) {
      double Arc2 =
          distTraveled(&rightEncoderFB); // rightEncoderFB travel, to forward
                                         // direction of robot is positive
      double Arc1 =
          distTraveled(&leftEncoderFB); // leftEncoderFB travel, to forward
                                        // direction of robot is positive
      double Arc3 = -distTraveled(&encoderLR);   // backEncoderFB travel, to right of robot is positive
      arc1g += Arc1;
      arc2g += Arc2;
      arc3g += Arc3;


      double a = 4.76775; // distance between two tracking wheels
      double b = 2.40625; // distance from tracking center to back tracking
                         // wheel, positive direction is to the front of tracking point
      double P1 = (Arc2 - Arc1);
      double Delta_y, Delta_x;

      //getting rotation and checking for error
      double i1 = inertial.get_rotation();
      double i2 = inertial2.get_rotation();
      static double prev_velw = 0;
      double radRotation;
      if (i1 == PROS_ERR_F) {
        radRotation = mod(2 * M_PI, ((-i2) + chaIntAng) * M_PI / 180);
      } else if (i2 == PROS_ERR_F) {
        radRotation = mod(2 * M_PI, ((-i1) + chaIntAng) * M_PI / 180);
      } else {
        radRotation = mod(2 * M_PI, ((-i1 - i2) / 2 + chaIntAng) * M_PI / 180);
      }
      robot.angle = radRotation * 180 / M_PI;

      //checking if calculated angle difference to inertial angle difference is too large and resetting if is too large
      double angle_error = odoHeading - radRotation;
      if (angle_error > M_PI) {
        angle_error -= 2 * M_PI;
      } else if (angle_error < -M_PI) {
        angle_error += 2 * M_PI;
      }
      // relying on heading calibrated by odometry in order to reduce noise but
      // also comparing it to inertial to check for drift
      if (fabs(angle_error) >= .05) {
        odoHeading = radRotation;
      }

      
      //refer to notebook for more in depth guide of odometry
      //tracks position be approximating a curve generated by the two arcs followed by the encoders
      double Delta_heading = P1 / a; // change of heading
      if (P1 !=
          0) { // if there are change of heading while moving, arc approximation
        double Radius_side =
            (Arc1 + Arc2) * a / (2 * P1); // radius to either side of the robot
        double Radius_back =
            Arc3 / Delta_heading - b; // radius to back or forward of the robot
        double cos_side = sin(odoHeading + Delta_heading) - sin(odoHeading);
        double cos_back = -cos(odoHeading + Delta_heading) + cos(odoHeading);
        double sin_side = -cos(odoHeading + Delta_heading) + cos(odoHeading);
        double sin_back = -sin(odoHeading + Delta_heading) + sin(odoHeading);

        Delta_x = Radius_side * cos_side + Radius_back * cos_back;
        Delta_y = Radius_side * sin_side + Radius_back * sin_back;

      } else {
        Delta_x =
            Arc1 * cos(odoHeading) + (Arc3 * cos(odoHeading - (M_PI / 2)));
        Delta_y =
            Arc1 * sin(odoHeading) + (Arc3 * sin(odoHeading - (M_PI / 2)));
      }
      odoHeading += Delta_heading;
      robot.odoangle +=Delta_heading*180/M_PI;
      odoHeading = mod(2 * M_PI, odoHeading);
      static float T = 0;
      static double previousT = 0;
      T = float(millis()) / 1000 - previousT;
      previousT += T;

      //outputting calculated values
      robot.odovelW = Delta_heading * 180 / (M_PI * T);
      robot.imuvelw =
          0.5 * (-inertial2.get_gyro_rate().z + inertial.get_gyro_rate().z);
      robot.velW = 0.5 * (robot.imuvelw + robot.odovelW);
      robot.velX = Delta_x / T;
      robot.velY = Delta_y / T;
      // when visOdom is working, change xpos to xposodom && same with ypos
      robot.xpos += Delta_x;
      robot.ypos += Delta_y;
      robot.angAccel = (robot.velW - prev_velw) / T;
      prev_velw = robot.velW;
      robot.odoxpos += Delta_x;
      robot.odoypos += Delta_y;

      // note:division of one hundred is due to the angle is messured in
      // centideg
      robot.turAng = robot.angle;
      while (robot.turAng > 360) {
        robot.turAng -= 360;
      }
      while (robot.turAng < 0) {
        robot.turAng += 360;
      }

      // delay to allow for other tasks to run
      delay(4);
    }
  }

  //speed and angle calculation for aimbot
  bool speedAngleCalc_bool = true;
  void speedAngleCalc(void) { 
    speedAngleCalc_bool = true;
    // acceleration due to gravity in inches per second
    float g = -386.08858267717;
    //angle that the disc exits to robot at
    float exitAngle = 2*M_PI/9;

    while (!competition::is_disabled() && speedAngleCalc_bool == true) {
     
      //calculating difference in position
      robotGoal.dx = goal.xpos - robot.xpos;
      robotGoal.dy = goal.ypos - robot.ypos;
      robotGoal.dz = goal.zpos - robot.zpos;

      double dist = sqrt(pow(robotGoal.dx,2) + pow(robotGoal.dy,2));

      //calculating exit velocity and 
      goalSpeed = dist /
                  (cos(exitAngle) * sqrt(2 * (robotGoal.dz-tan(exitAngle)*dist) / g ));
      
      goalAngle = atan2(robotGoal.dy,robotGoal.dx)*180/M_PI;

      //checking how many discs are in the magazine
      double angle = potentiometer.get_angle();
      if (angle > 220) {
        robot.magFullness = 0;
      } else if (angle >210) {
        robot.magFullness = 1;
      } else if (angle > 200) {
        robot.magFullness = 2;
      } else {
        robot.magFullness = 3;
      }
      
      delay(optimalDelay);
    }
  }

  bool underRoller(int sensorNum) {
    if (sensorNum == 1) {
      if (leftOpticalSensor.get_proximity() ==255) {
        return 1;
      } else {
        return 0;
      }
    } else {
      if (rightOpticalSensor.get_proximity() == 255) {
        return 1;
      } else {
        return 0;
      }
    }
  }

  bool rollerIsGood(int fwd, bool newRoller = false) {
    static bool seenBad = 0;
    bool isGood = false;
    if (newRoller){
      seenBad = 0;
    }
    if (!underRoller(1) && !underRoller(2)){
      return 0;
    }
    if (underRoller(1)) {
      c::optical_rgb_s color_sensor = leftOpticalSensor.get_rgb();
      logValue("r", color_sensor.red, 0);
      logValue("b", color_sensor.blue, 1);
      logValue("prox", leftOpticalSensor.get_proximity(), 25);
      if (((color_sensor.red > 400 && color == true) ||
           (color_sensor.red < 300 && color == false)) &&
          underRoller(1)) {
        isGood = true;
      } else {
        isGood = false;
      }
    } else{
      c::optical_rgb_s color_sensor = rightOpticalSensor.get_rgb();
      logValue("r", color_sensor.red, 23);
      logValue("b", color_sensor.blue, 24);
      logValue("prox", rightOpticalSensor.get_proximity(), 25);

      if (((color_sensor.red > 400 && color == true) ||
           (color_sensor.red < 300 && color == false)) &&
          underRoller(-1)) {
        isGood = true;
      } else {
        isGood = false;
      }
    }

    if (isGood == false && seenBad == false && underRoller(fwd)){
      seenBad = true;
    }

    if (seenBad == true && isGood == true){
      return 1;
    }
    else{
      return 0;
    }
  }

  Object prevShotRobot;
  double prevAngularVelocityShot;
  void positionCorrection(void){
    double changeX = robot.xpos - prevShotRobot.xpos;
    double changeY = robot.ypos - prevShotRobot.ypos;

    double trueX[2] = {0,0};
    double trueY[2]{0,0};
    double trueAngle[2]{0,0};

    //checking which is better

    bool secondGood;

    robot.xpos = trueX[secondGood] + changeX;
    robot.ypos = trueY[secondGood] + changeY;
    chaIntAng = chaIntAng + (trueAngle[secondGood] - prevShotRobot.angle);
  }
  
  void shooting(int velocity){
    static double prevAngularVelocity = velocity;
    static int prevTime = millis();
    static Object prevRobotData = robot;

    //(rotations per second) per milisecond
    double velocityDerivative = (velocity - prevAngularVelocity)/(millis() - prevTime);

    if (velocityDerivative < -2){
      prevShotRobot = prevRobotData;
      prevAngularVelocityShot = velocity;
    }

    prevRobotData = robot;
    prevAngularVelocity = velocity;
    prevTime = millis();
  }
};

extern void odometry_Wrapper(void *sensing);
extern void GPS_Wrapper(void *sensing);
extern void odometry_Wrapper(void *sensing);
extern void speedAngleCalc_Wrapper(void *sensing);
extern sensing_t sensing;

#endif