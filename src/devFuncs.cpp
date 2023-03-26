#include "main.h"
#include "output.h"
#include "pros/distance.hpp"

using namespace pros;
class tunedSystems_t
{
public:
    PID_t driveFR, driveSS, turret, flyWheel;
} PID;

//outputting distances and colors for update at tournament
void tuneSensors(void){
    
    Optical opticalSensor(14);
    Optical opticalSensor2(11);
    Distance distSense(20);
    while(1){
        c::optical_rgb_s color_sensor = opticalSensor.get_rgb();
                    
        logValue("1r", color_sensor.red, 0);
        logValue("1b", color_sensor.blue, 1);
        logValue("1d", opticalSensor.get_proximity(), 2);
        color_sensor = opticalSensor2.get_rgb();

        logValue("2r", color_sensor.red, 3);
        logValue("2b", color_sensor.blue, 4);
        logValue("2d", opticalSensor2.get_proximity(), 5);

        logValue("dist", distSense.get(), 6);
        delay(20);
    }
    
    
}