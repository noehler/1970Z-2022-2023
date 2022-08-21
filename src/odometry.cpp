// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"
#define DEG2RAD M_PI/360

position_t robot;
position_t homeGoal;
robotGoalRelatives robotGoal;

//function to automatically detect distance covered by an individual tracking wheel
//using pointers so that I can determine what ratio is needed to convert from degrees to distance without using a second variable
double distTraveled(ADIEncoder * encoderLoc, bool resetEncoder = true){
    double ratioNum;
    if ((encoderLoc == &leftEncoderFB) || (encoderLoc == &rightEncoderFB) || (encoderLoc == &encoderLR)){
        ratioNum=1.320;
    }
    else{
        ratioNum = 0;
    }


    //transfering from memory location to value at that memory location
    ADIEncoder encoderVar = *encoderLoc;

    double degreesTraveled = encoderVar.get_value();
    if (resetEncoder == true){
        encoderVar.reset();
    }

    double distTraveled = degreesTraveled * ratioNum;
    
    return distTraveled;
}
 
void basicOdometry(void){
    //getting average of fwd reverse distance traveled to account for turning
    double avgFR = (distTraveled(&leftEncoderFB) + distTraveled(&rightEncoderFB))/2;
    double avgSS = distTraveled(&encoderLR);

    //converting from relative to robot movement to absolute to field movement
    double changeX = avgFR * cos(inertial.get_heading()*DEG2RAD) + avgSS * cos((inertial.get_heading()+90)*DEG2RAD);
    double changeZ = avgFR * sin(inertial.get_heading()*DEG2RAD) + avgSS * sin((inertial.get_heading()+90)*DEG2RAD);

    //adding calculated values onto the global position of the robot
    robot.xpos += changeX;
    robot.zpos += changeZ;
}

