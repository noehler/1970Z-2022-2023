#ifndef __ROBOTCONFIG_H__
#define __ROBOTCONFIG_H__

//will always end up being false but makes edditor realize that api.h is seen
#ifndef _PROS_MAIN_H_
#include "api.h"
#endif

using namespace pros;

class Object{
    public:
        double xpos, ypos,zpos,angle;
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
    public:
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
                    if (master.get_digital(DIGITAL_B)){
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