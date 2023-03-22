#include "main.h"
#include "output.h"
#include "pros/distance.hpp"

using namespace pros;
class tunedSystems_t
{
public:
    PID_t driveFR, driveSS, turret, flyWheel;
} PID;

void tuneSensors(void){
    
    Optical opticalSensor(14);
    Optical opticalSensor2(11);
    Distance distSense(20);
    while(1){
        c::optical_rgb_s color_sensor = opticalSensor.get_rgb();
                    
        logValue("1r", color_sensor.red, 0);
        logValue("1b", color_sensor.blue, 1);
        color_sensor = opticalSensor2.get_rgb();

        logValue("2r", color_sensor.red, 2);
        logValue("2b", color_sensor.blue, 3);

        logValue("dist", distSense.get(), 4);

    }
    
    
}