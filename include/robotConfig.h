#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

#include "api.h"
#include "GUI.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "output.h"
#include "Autons/autonSetup.h"

using namespace pros;

extern Controller master;
extern Controller sidecar;

extern bool highTurretInitAng;

extern double getNum(std::string Output);

class Object{
    public:
        double xpos, ypos,zpos,
        odoxpos, odoypos, 
        GPSxpos, GPSypos, 
        inertialxpos, inertialypos,
        angle, turAng, turvelw,
        velX, velY, velW, turvelocity ,odovelW , imuvelw, 
        angAccel, xAccel, yAccel;
        bool turretLock = false;
        int magFullness;
};

extern double targetAngleOffest;
extern double chaIntAng;
extern double goalAngle;
class sensing_t{
    private:
        //JOHN: When calibrating make sure that the id(first arguement) is 1
        vision_signature_s_t REDGOAL = Vision::signature_from_utility(1, 4499, 8193, 6346, -1589, -429, -1009, 2.5, 0);
        //JOHN: When calibrating make sure that the id(first arguement) is 2
        vision_signature_s_t BLUEGOAL = Vision::signature_from_utility(2, -2553, -1927, -2240, 1009, 3205, 2107, 2.4, 0);

        class robotGoalRelatives {
        public:
            //storing the absolute and relative horizontal angle between goal and robot
            double dx, dy,dz;
        }robotGoal;

        //using pointers so that I can determine what ratio is needed to convert from degrees to distance without using a second variable
        double distTraveled(ADIEncoder * encoderLoc, bool resetEncoder = true){
            double radius;
            if (encoderLoc == &leftEncoderFB){
                radius=1.375;
            }
            if (encoderLoc == &rightEncoderFB){
                radius=1.375;
            }
            else{
                radius = 1.375;
            }

            double degreesTraveled = encoderLoc->get_value();

            if (resetEncoder == true){
                encoderLoc->reset();
            }

            double distTraveled = (degreesTraveled/360) * radius * 2 * M_PI;

            return distTraveled;
        }

        double magnitude(double a,double b){
            return sqrt(pow(a,2)+pow(b,2));
        }

        double mod(double base, double var){
            while (base<var){//e.g. 361mod360 = 1
                var-=base;
            }
            while (var<0){ // e.g. -361mod 360 = 359
                var+=base;
            }
            return var;
        }

        double turretPosChoice(double angBetween){
            static int prevBadTime = c::millis();
            return angBetween;
        }

        int optimalDelay = 20;
    public:
    
        ADIEncoder leftEncoderFB;
        ADIEncoder rightEncoderFB;
        ADIEncoder encoderLR;
        Rotation turretEncoder;
        Imu inertial2;

        ADIAnalogIn upLoaded;
        ADIAnalogIn deckLoaded;
        ADIAnalogIn holeLoaded;

        Imu inertial;

        Optical opticalSensor;
        Optical opticalSensor2;
        Distance distSense;

        Vision discSearch;

        GPS GPS_sensor;
    
        Object robot;
        Object goal;
        double goalSpeed = 0;
        bool magFull;
        
        sensing_t(void):leftEncoderFB({{9,'E','F'}, true}), rightEncoderFB({{9,'C', 'D'},true }),
                        encoderLR({{9,'A','B'}, true}), turretEncoder(8), inertial2(7), upLoaded({22,'E'}),
                        deckLoaded({22,'C'}), holeLoaded({22,'G'}), inertial(21), opticalSensor(14), opticalSensor2(11),
                        discSearch(3), distSense(20), GPS_sensor(12){}

        void Init(void){
            if (double(turretEncoder.get_position())/100 > 180){
                highTurretInitAng = true;
            }
            else{
                highTurretInitAng = false;
            }

            std::cout << "values set\n";
            static bool inertialsSet = false;
            if (!inertialsSet){
                inertial.reset();
                inertial2.reset();
                int startTime = 0;
                while (inertial.is_calibrating()  || inertial2.is_calibrating()){
                    if (millis() - startTime > 3000){
                        master.clear_line(2);
                        delay(50);
                        master.print(2, 0, "CalibrationFailing");
                    }
                    if (millis() - startTime > 3000 && master.get_digital(E_CONTROLLER_DIGITAL_B)){
                        break;
                    }
                    delay(50);
                    std::cout << "calibrating\n";
                }
                delay(1000);
                std::cout << "calibrated\n";
                inertialsSet = true;
            }
            GPS_sensor.set_offset(0, 0);

            inertial.set_heading(0);
            inertial2.set_heading(0);

            master.print(2, 0, "Calibration Done");
            //sensor failure output here.
        }
        //color = true > red. color = false > blue.
        void set_status (double xpos, double ypos, double heading ,double color_sensor_luminance , int colorPT){
            opticalSensor.set_led_pwm(color_sensor_luminance);
            opticalSensor2.set_led_pwm(color_sensor_luminance);
            robot.xpos = xpos;
            robot.ypos = ypos;
            robot.odoxpos = xpos;
            robot.odoypos = ypos;
            robot.zpos = 8.5;
            chaIntAng = heading;
            color = colorPT; 
            goal.xpos = 20;
            goal.ypos = 124;
            goal.zpos = 30;
        }

        void GPS_tracking(void){
            //note: rotation of Gps strip can vary depend on field if true, going to verify on 2/28
            bool sensorFail = false;
            while (1){
                static bool prevX;
                static bool prevY;
                static double ydiff = 0, xdiff = 0;//diff from middle/gps origin
                pros::c::gps_status_s_t temp_status = GPS_sensor.get_status();
                if (color!=0){
                    xdiff = double(temp_status.y)*39.37;
                    ydiff = -double(temp_status.x)*39.37;
                } else{
                    xdiff = -double(temp_status.y)*39.37;
                    ydiff = double(temp_status.x)*39.37;
                }

                if (GPS_sensor.get_error() < 0.012 && 
                    sqrt(pow(robot.velX,2) + pow(robot.velY,2)) < 1 && 
                    robot.xpos >36 && robot.xpos < 108 && robot.ypos >36 && robot.ypos < 108 && !sensorFail)
                {
                    //robot.xpos = robot.GPSxpos;
                    //robot.ypos = robot.GPSypos;
                }
                else{
                    /*robot.xpos = robot.odoxpos;
                    robot.ypos = robot.odoypos;*/
                }
                static bool firstFail = true;
                // if red: x direction is correct y is flipped, if blue: x direction is flipped, y is correct
                if (prevX - (72 + xdiff) == 0){
                    sensorFail = true;

                    if (firstFail){
                        master.clear_line(2);
                        delay(50);
                        master.print(2, 0, "GPS Fail");
                        firstFail = false;
                    }
                }
                else{
                    sensorFail = false;
                }
                prevX = (72 + xdiff);
                robot.GPSxpos = 72 + xdiff - cos(robot.turAng*M_PI/180)*4.5;
                robot.GPSypos = 72 + ydiff - sin(robot.turAng*M_PI/180)*4.5;

                delay(20);
            }
        }
        void odometry(void){
            while (1){
                static double odoHeading = 0;
                static double odomposx = 0;
                static double odomposy = 0;
                double Arc1 = distTraveled(&rightEncoderFB); //rightEncoderFB travel, to forward direction of robot is positive
                double Arc2 = distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positiv
                double Arc3 = -distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
                double a = 4.8125; //distance between two tracking wheels
                double b = -3.625; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
                double P1 = (Arc1 - Arc2);
                double Delta_y, Delta_x;
                double i1 = inertial.get_rotation();
                double i2 = inertial2.get_rotation();
                static double prev_velw =0;
                double radRotation;
                if (i1 == PROS_ERR_F)
                {
                    radRotation = mod(2*M_PI,((-i2)+chaIntAng)*M_PI/180);
                }
                else if (i2 == PROS_ERR_F)
                {
                    radRotation = mod(2*M_PI,((-i1)+chaIntAng)*M_PI/180);
                }
                else{
                    radRotation = mod(2*M_PI,((-i1-i2)/2+chaIntAng)*M_PI/180);
                }
                robot.angle = radRotation*180/M_PI;
                //std::cout<<"\nradRotation"<<radRotation*180/M_PI<<"Initang"<<chaIntAng;
                double angle_error = odoHeading - radRotation;
                if (angle_error > M_PI){
                    angle_error -=2*M_PI;
                } else if (angle_error<-M_PI){
                    angle_error +=2*M_PI;
                }
                // relying on heading calibrated by odometry in order to reduce noise but also comparing it to inertial to check for drift
                if (fabs(angle_error) >= .05){
                    odoHeading = radRotation;
                    //std::cout << "\n chassis heading error"<<angle_error;
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
                //this is off until I can trust odometry again
                //robot.angle = odoHeading*180/M_PI;
            static float T = 0;
            static double previousT =0;
            T = float(millis())/1000 - previousT;
            previousT+=T;
            robot.odovelW = Delta_heading*180/(M_PI*T);
            robot.imuvelw = 0.5*(-inertial2.get_gyro_rate().z+inertial.get_gyro_rate().z);
            robot.velW = 0.5*(robot.imuvelw+robot.odovelW);
            robot.velX = Delta_x/T;
            robot.velY = Delta_y/T;
            //when visOdom is working, change xpos to xposodom && same with ypos
            robot.xpos += Delta_x;
            robot.ypos += Delta_y;
            robot.angAccel = (robot.velW-prev_velw)/T;
            prev_velw = robot.velW;
            robot.odoxpos += Delta_x;
            robot.odoypos += Delta_y;
            //std::cout << "\n" << -inertial2.get_gyro_rate().z/180*M_PI;
            //std::cout << "\n" << inertial.get_gyro_rate().z/180*M_PI;
            robot.turvelw = double(turretEncoder.get_velocity())/100;
            //note:division of one hundred is due to the angle is messured in centideg
            robot.turAng = double(turretEncoder.get_angle())/100 +180+ robot.angle;
            while (robot.turAng > 360){
                robot.turAng -= 360;
            }
            while (robot.turAng < 0){
                robot.turAng += 360;
            }

            //delay to allow for other tasks to run
            delay(5);
            }
        }

        bool SSOSTTT_bool = true;
        void SSOSTTT(void){//singSameOldSongTimeTurretTwister       //(itterative Turret Angle calculation)
            SSOSTTT_bool = true;
            //acceleration due to gravity in inches per second
            float g = 386.08858267717;
            // constant based around acceleration due to gravity in inches per second devided squared by four -(g^2)/4
            double a = -37266.393609;
            targetAngleOffest = 0;
            //chaIntAng = 0;
            while(!competition::is_disabled() && SSOSTTT_bool == true){
                //define quartic equation terms  
                
                robotGoal.dx = goal.xpos - robot.xpos;
                robotGoal.dy = goal.ypos - robot.ypos;
                robotGoal.dz = goal.zpos - robot.zpos;
                double c = pow(robot.velX, 2) + pow(robot.velY, 2) - robotGoal.dz * g;
                double d = -2 * robotGoal.dx * robot.velX - 2 * robotGoal.dy * robot.velY;
                double e = pow(robotGoal.dx, 2) + pow(robotGoal.dy, 2) - pow(robotGoal.dz, 2);
                double D = 1000000000000;
                double T = 0.1;
                bool close_enough = false;
                while (close_enough != true){
                    if (D > 10000){
                    T += 0.1;
                    }  
                    else if (D > 1){
                    T += 0.001;
                    }
                    else{
                    close_enough = true;
                    }
                    D = a * pow(T, 4) + c * pow(T, 2) + d * T + e;
                }
                
                double P1 = robotGoal.dy - robot.velY * T;
                double P2 = robotGoal.dx - robot.velX * T;
                double Tar_ang = 0;
                if (P2 == 0){
                    if (P1 > 0){
                    Tar_ang = M_PI/2;
                    } else {
                    Tar_ang = -M_PI/2;
                    }
                } else {
                    Tar_ang = atan(P1 / P2);
                    if (P2 < 0){
                    Tar_ang = Tar_ang + M_PI;
                    } else {
                    Tar_ang = Tar_ang +2*M_PI;
                    }
                }
                double P3 = cos(Tar_ang) * 0.707106781187 * T;
                double V_disk = P2 / P3;
                double turOfCenterOffset = 0; // offcenter offset, not tested yet
                //outputting calculated values

                
                double dist = distSense.get();
                if (dist == PROS_ERR){
                master.print(2, 0, "DistFailed");
                robot.magFullness = 3;
                }
                else if (dist < 40){
                    robot.magFullness = 3;
                }
                else if (dist < 80){
                    robot.magFullness = 2;
                }
                else if (dist < 100){
                    robot.magFullness = 1;
                }
                else{
                    robot.magFullness = 0;
                }
                if (V_disk < 160){
                    V_disk = 160;
                }
                goalSpeed = V_disk;
                if (!robot.turretLock){
                    goalAngle = turretPosChoice(Tar_ang *180/M_PI + 0*targetAngleOffest+0*turOfCenterOffset);
                }
                else{
                    goalAngle = robot.angle + 180;
                }

                while (goalAngle > 180){
                    goalAngle -= 360;
                }
                while (goalAngle < -180){
                    goalAngle += 360;
                }
                robot.turvelocity = (robot.velX*P1-robot.velY*P2)/(pow(P1,2)+pow(P2,2))*180/M_PI;
                delay(optimalDelay);
            }
        }

        bool underRoller(int sensorNum){
            if (sensorNum == 1){
                if (opticalSensor.get_proximity() > 200){
                    return 1;
                }
                else{
                    return 0;
                }
            }
            else{
                if (opticalSensor2.get_proximity() > 200){
                    return 1;
                }
                else{
                    return 0;
                }
            }
            
        }

        bool rollerIsGood(void){
            if(underRoller(1)){
                c::optical_rgb_s color_sensor = opticalSensor.get_rgb();
                logValue("r", color_sensor.red,24);
                logValue("b", color_sensor.blue,25);
                logValue("prox", opticalSensor.get_proximity(),26);
                if (((color_sensor.red > 3800 && color == true) || (color_sensor.red < 1200 && color == false)) && underRoller(1)){
                    return 1;
                }
                else{
                    return 0;
                }
            }else if(underRoller(2)){
                c::optical_rgb_s color_sensor = opticalSensor2.get_rgb();
                logValue("r", color_sensor.red,24);
                logValue("b", color_sensor.blue,25);
                logValue("prox", opticalSensor2.get_proximity(),26);
                
                if (((color_sensor.red > 2000 && color == true) || (color_sensor.red < 500 && color == false)) && underRoller(2)){
                    return 1;
                }
                else{
                    return 0;
                }
            }
            else{
                c::optical_rgb_s color_sensor = opticalSensor2.get_rgb();
                logValue("r", 0,24);
                logValue("b", 0,25);
                logValue("prox", opticalSensor2.get_proximity(),26);
                return 0;
            }
            
        }
};

extern void odometry_Wrapper(void* sensing);
extern void GPS_Wrapper(void* sensing);
extern void odometry_Wrapper(void* sensing);
extern void SSOSTTT_Wrapper(void* sensing);
extern sensing_t sensing;


#endif 