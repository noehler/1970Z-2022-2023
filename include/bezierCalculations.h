#ifndef __BEZIERCALCULATIONS_H__
#define __BEZIERCALCULATIONS_H__

#include "robotConfig.h"
#include "output.h"
class bez_Return_t{
    public:
        bez_Return_t(void){
            for(int i = 0; i < 100; i++){
                returnPoints[i][0] = 0;
                returnPoints[i][1] = 0;
            }
        }
        double basePoints[10][2];
        double returnPoints[100][2];
        double returnVelocity[100];
        double radius[100];
        int length = 0;
};

class beziers_t{
    private:
    public:
        //function used to generate more smooth path for auton movement length is used to signify the max length of the string, precision is the amount of points generated between start and finish
        //length will be increased by one to account for robot position different from 1st point
        bez_Return_t generatePath(double inputPoints[10][2], int length, int precision){    
            //accounting for robot being away from starting point
            double oneDiff = sqrt(pow(sensing.robot.xpos - inputPoints[0][0],2) + pow(sensing.robot.ypos - inputPoints[0][1],2));
            oneDiff = 0;
            if (oneDiff > 0){
                for (int i = length; i > 5; i--){
                    inputPoints[i][0] = inputPoints[i-1][0];
                    inputPoints[i][1] = inputPoints[i-1][1];
                    delay(2);
                }
                inputPoints[0][0] = sensing.robot.xpos;
                inputPoints[0][1] = sensing.robot.ypos;
                length+=1;
            }
            
            bez_Return_t temp;
            for (int i = 0; i< 10; i++){
                temp.basePoints[i][0] = inputPoints[i][0];
                temp.basePoints[i][1] = inputPoints[i][1];
            }
            temp.length = precision;

            double vectors[100][2];
            for (int i = 0; i < precision; i++){
                double t = double(i)/precision;
                double weight[length];
                if (length == 4){
                    weight[0] = 1-3*t + 3*pow(t,2) - pow(t,3);
                    weight[1] = 3*t - 6*pow(t,2) + 3 * pow(t,3);
                    weight[2] =  3*pow(t,2)-3*pow(t,3);
                    weight[3] = pow(t,3);
                    
                }
                else if (length == 5){
                    weight[0] = 1-4*t + 6*pow(t,2) - 4*pow(t,3) + pow(t,4);
                    weight[1] = 4*t - 12*pow(t,2) + 12 * pow(t,3) - 4*pow(t,4);
                    weight[2] =  6*pow(t,2)-12*pow(t,3)+6*pow(t,4);
                    weight[3] = -4*pow(t,4)+4*pow(t,3);
                    weight[4] = pow(t,4);
                }
                else if (length == 6){
                    weight[0] = 1 - 5*t + 10*pow(t,2) - 10*pow(t,3) + 5*pow(t,4) - pow(t,5);
                    weight[1] = 5*t - 20*pow(t,2) + 30*pow(t,3) - 20*pow(t,4) + 5*pow(t,5);
                    weight[2] = 10*pow(t,2) - 30 * pow(t,3) + 30*pow(t,4) - 10*pow(t,5);
                    weight[3] = 10*pow(t,3) - 20*pow(t,4) + 10*pow(t,5);
                    weight[4] = 5*pow(t,4) - 5*pow(t,5);
                    weight[5] = pow(t,5);
                }
                else if (length == 7){
                    weight[0] = 1 - 6*t + 15*pow(t,2) - 20*pow(t,3) + 15*pow(t,4) - 6*pow(t,5) + pow(t,6);
                    weight[1] = 6*t - 30*pow(t,2) + 60*pow(t,3) - 60*pow(t,4) + 30*pow(t,5) - 6*pow(t,6);
                    weight[2] = 15*pow(t,2) - 60*pow(t,3) + 90*pow(t,4) - 60*pow(t,5) + 15*pow(t,6);
                    weight[3] = 20*pow(t,3) - 60 * pow(t,4) + 60*pow(t,5) - 20*pow(t,6);
                    weight[4] = 15*pow(t,4) - 30*pow(t,5) + 15*pow(t,6);
                    weight[5] = 6*pow(t,5) - 6*pow(t,6);
                    weight[6] = pow(t,6);
                    
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
                
                if (i > 0){
                    double dx = temp.returnPoints[i][0] - temp.returnPoints[i-1][0];
                    double dy = temp.returnPoints[i][1] - temp.returnPoints[i-1][1];
                    double dS = sqrt(pow(dx, 2) + pow(dy, 2));
                    temp.returnVelocity[i] = dS*(precision);
            
                    vectors[i][0] = dx*precision/temp.returnVelocity[i];
                    vectors[i][1] = dy*precision/temp.returnVelocity[i];

                    if (i == 1){
                        vectors[0][0] = dx*precision/temp.returnVelocity[i];
                        vectors[0][1] = dy*precision/temp.returnVelocity[i];
                    }
                    
                    double dVectorX = (vectors[i][0] - vectors[i-1][0])/dS;
                    double dVectorY = (vectors[i][1] - vectors[i-1][1])/dS;

                    temp.radius[i] = 1/sqrt(pow(dVectorX, 2) + pow(dVectorY, 2));

                    if (isnanf(temp.radius[i])){
                        temp.radius[i] = 1000000;
                    }
                    if (isnanf(temp.returnVelocity[i])){
                        temp.returnVelocity[i] = 100;
                    }
                }
                

                logValue("xG", temp.returnPoints[i][0], 0);
                logValue("yG", temp.returnPoints[i][1], 1);
                outValsSDCard();
                
            }

            return temp;
        }
        
};
extern beziers_t beziers;
#endif