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
    
    Optical leftOpticalSensor(12);
    Optical rightOpticalSensor2(14);
    while(1){
        c::optical_rgb_s color_sensor = leftOpticalSensor.get_rgb();
                    
        logValue("Lr", color_sensor.red, 0);
        logValue("Lb", color_sensor.blue, 1);
        logValue("1d", leftOpticalSensor.get_proximity(), 2);
        color_sensor = rightOpticalSensor2.get_rgb();

        logValue("2r", color_sensor.red, 3);
        logValue("2b", color_sensor.blue, 4);
        logValue("2d", rightOpticalSensor2.get_proximity(), 5);

        delay(20);
    }
    
    
}