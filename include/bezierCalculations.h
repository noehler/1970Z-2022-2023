#ifndef __BEZIERCALCULATIONS_H__
#define __BEZIERCALCULATIONS_H__

#include "robotConfig.h"
#include "sdLogging.h"
class bez_Return_t{
    public:
        double returnPoints[100][2];
        int length;
};

class beziers_t{
    private:
    public:
        //function used to generate more smooth path for auton movement length is used to signify the max length of the string, precision is the amount of points generated between start and finish
        //length will be increased by one to account for robot position different from 1st point
        bez_Return_t generatePath(double inputPoints[10][2], int length, int precision){            
            //accounting for robot being away from starting point
            double oneDiff = sqrt(pow(sensing.robot.xpos - inputPoints[0][0],2) + pow(sensing.robot.ypos - inputPoints[0][1],2));
            if (oneDiff > 5){
                for (int i = length; i > 0; i--){
                    inputPoints[i][0] = inputPoints[i-1][0];
                    inputPoints[i][1] = inputPoints[i-1][1];
                    delay(2);
                }
                inputPoints[0][0] = sensing.robot.xpos;
                inputPoints[0][1] = sensing.robot.ypos;
                length+=1;
            }
            
            bez_Return_t temp;
            temp.length = precision;

            for (int i = 0; i < precision; i++){
                double t = double(i)/precision;
                double weight[length];
                if (length == 4){
                    for (int i = 0; i < 40; i++){   
                        logValue("error", 00, i);
                    }
                    outValsSDCard();
                    break;

                }
                else if (length == 5){
                    weight[0] = 1-4*t + 6*pow(t,2) - 4*pow(t,3) + pow(t,4);
                    weight[1] = 4*t - 12*pow(t,2) + 12 * pow(t,3) - 4*pow(t,4);
                    weight[2] =  6*pow(t,2)-12*pow(t,3)+6*pow(t,4);
                    weight[3] = -4*pow(t,4)+4*pow(t,3);
                    weight[4] = pow(t,4);
                }
                else if (length == 6){
                    for (int i = 0; i < 40; i++){   
                        logValue("error", 00, i);
                    }
                    outValsSDCard();
                    break;
                }
                else{
                    for (int i = 0; i < 40; i++){
                        logValue("error", 00, i);
                    }
                    outValsSDCard();
                    break;
                }
                
                for (int j = 0; j < length; j++){
                    temp.returnPoints[i][0] += weight[j] * inputPoints[j][0];
                    temp.returnPoints[i][1] += weight[j] * inputPoints[j][1];
                }

            }

            return temp;
        }
        
};
extern beziers_t beziers;
#endif