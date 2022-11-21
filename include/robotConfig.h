#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

//will always end up being false but makes edditor realize that api.h is seen
#ifndef _PROS_MAIN_H_
#include "api.h"
#endif

using namespace pros;

class Object{
    public:
        double xpos, ypos,zpos,angle, velX, velY, velW;
};

extern Controller master;
extern Controller sidecar;

class sensing_t{
    private:
        ADIEncoder leftEncoderFB;
        ADIEncoder rightEncoderFB;
        ADIEncoder encoderLR;
        Rotation turretEncoder;
        Imu inertialTurret;

        ADIAnalogIn upLoaded;
        ADIAnalogIn deckLoaded;
        ADIAnalogIn holeLoaded;

        Imu inertial;

        Optical opticalSensor;

        Vision turVisionL;
        Vision turVisionR;
        vision_signature_s_t REDGOAL;
        vision_signature_s_t BLUEGOAL;

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

    public:
    
        Object robot;
        Object goal;
        double chaIntAng = 0;

        sensing_t(void):leftEncoderFB({{9,'C','D'}, true}), rightEncoderFB({{9,'E', 'F'},true }),
                        encoderLR({{9,'A','B'}}), turretEncoder(10), inertialTurret(12), upLoaded({22,'F'}),
                        deckLoaded({9,'H'}), holeLoaded({22,'E'}), inertial(8), opticalSensor(18),
                        turVisionL(15), turVisionR(19)
        {
            int startTime = millis();
            while (inertial.is_calibrating()  || inertialTurret.is_calibrating()){
                std::cout << "\nCalibrating!";
                delay(40);
                if (millis() - startTime > 3000){
                    delay(50);
                    master.clear();
                    delay(50);
                    master.print(2,1,"Calibration Failing.");
                    delay(50);
                    master.print(1,1,"B to ignore.");
                    if (master.get_digital(E_CONTROLLER_DIGITAL_B)){
                        break;
                    }
                }
            }
            inertial.set_heading(0);
            inertialTurret.set_heading(0);
            delay(50);
            master.clear();
            delay(50);
            master.print(1,1,"Calibration Success.");
        }

        void odometry(void){
            while (1){
                static double odoHeading = 0;
                static double odomposx = 0;
                static double odomposy = 0;
                double Arc1 =distTraveled(&rightEncoderFB); //rightEncoderFB travel, to forward direction of robot is positive
                double Arc2 =distTraveled(&leftEncoderFB); //leftEncoderFB travel, to forward direction of robot is positiv
                double Arc3 = distTraveled(&encoderLR); //backEncoderFB travel, to right of robot is positive
                double a = 8; //distance between two tracking wheels
                double b = -2.5; //distance from tracking center to back tracking wheel, positive direction is to the back of robot
                double P1 = (Arc1 - Arc2);
                double Delta_y, Delta_x;
                double radRotation = mod(2*M_PI,(-inertial.get_heading()+chaIntAng)*M_PI/180);
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
            robot.velW = Delta_heading/T;
            robot.velX = Delta_x/T;
            robot.velY = Delta_y/T;
            //when visOdom is working, change xpos to xposodom && same with ypos
            robot.xpos += Delta_x;
            robot.ypos += Delta_y;
            }
        }

        void SSOSTTT(void){//singSameOldSongTimeTurretTwister       //(itterative Turret Angle calculation)
            while(!competition::is_disabled()){
                
            }
        }

};

extern void odometry_Wrapper(void* sensing);
extern void SSOSTTT_Wrapper(void* sensing);


#endif