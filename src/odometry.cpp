// made on July 28, 2022 by Nathaniel Oehler

#include "main.h"

position_t robot;
position_t homeGoal;
robotGoalRelatives robotGoal;

enum wheelRatio_t{ leftEncoder, rightEncoder, sideSideEncoder, Drive};

double distTraveled(ADIEncoder encoder, wheelRatio_t ratioName, bool resetEncoder = false){
    double ratioNum;
    if ((ratioName == leftEncoder) || (ratioName == rightEncoder) || (ratioName == sideSideEncoder)){
        ratioNum=10;
    }
    else{
        ratioNum = 0;
    }

    double degreesTraveled = encoder.get_value();
    if (resetEncoder == true){
        encoder.reset();
    }

    double distTraveled = degreesTraveled * ratioNum;
    
    return distTraveled;
}