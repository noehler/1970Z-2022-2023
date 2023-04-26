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
    Optical rightOpticalSensor(14);
    while(1){
        leftOpticalSensor.set_led_pwm(100);
        rightOpticalSensor.set_led_pwm(100);
                    
        logValue("Lh", leftOpticalSensor.get_hue(), 0);
        logValue("Rh", leftOpticalSensor.get_hue(), 1);
        logValue("ld", leftOpticalSensor.get_proximity(), 2);
        logValue("rd", rightOpticalSensor.get_proximity(), 3);

        delay(20);
    }
    
    
}