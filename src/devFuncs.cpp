#include "main.h"
#include "motorControl.h"
#include "pros/motors.hpp"

using namespace pros;

class PID_t{
    public:
        double p, i, d, p2, i2,d2;
};
class tunedSystems_t{
    public:
        PID_t driveFR, driveSS, turret, flyWheel;
} PID;

void flyTuner(Motor flyWheel1, Motor flyWheel2, int loopDelay){
    motorControl_t mc;
    int PIDPOS = 1;
    double bestPVal = 94000;
    bool bsSet = 0;
    double prevP = PID.flyWheel.p;
    double prevI = PID.flyWheel.i;
    double prevD = PID.flyWheel.d;
    flyWheel1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    flyWheel2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    while (1){
        int gS = 500;
        int moveDir = -1;

        bool testDone = false;
        int startTime = millis();
        double PVal = 0;
        while (testDone == false){
            //std::cout << "fly\n";
            static double IPIDang = 0;
            if (competition::is_disabled()){
                IPIDang = 0;
            }
            double flyWVolt;
            double flyWheelW =(flyWheel2.get_actual_velocity());
            double diffFlyWheelW = gS-flyWheelW;
            static double prevFWdiffSPD = gS;

            IPIDang += diffFlyWheelW;
            double prop = PID.flyWheel.p*diffFlyWheelW;
            double integ = IPIDang*PID.flyWheel.i;
            double deriv = PID.flyWheel.d*(diffFlyWheelW - prevFWdiffSPD);
            flyWVolt = 12000.0/127*(prop + integ + deriv);

            PVal += fabs(diffFlyWheelW) + fabs(diffFlyWheelW - prevFWdiffSPD);

            prevFWdiffSPD = diffFlyWheelW;


            if (flyWVolt > 12000){
                flyWVolt = 12000;
            }
            if (flyWVolt < -12000){
                flyWVolt = -12000;
            } 
            if (fabs(diffFlyWheelW)<1&&flyWVolt==0){
                IPIDang = 0;
            }

            flyWheel2.move_voltage(flyWVolt);

            delay(loopDelay);
            if (millis() - startTime > 10000){
                std::cout << "\n\nPVal: " << PVal << "\nbVal: " << bestPVal << "\n";
                testDone = true;
                flyWheel2.brake();
            }
        }
        if (!bsSet){
            bestPVal = PVal;
            bsSet = true;
            std::cout << "\n\nBVal: " << bestPVal << "\n";
        }
        else{
            if (PVal > bestPVal){
                std::cout << "fail\n";
                double diff = (bestPVal - PVal) / bestPVal;
                if (PIDPOS == 1){
                    PID.flyWheel.p = prevP;
                    PID.flyWheel.i *= 1.0-diff*moveDir;
                    std::cout << "\nIA\n";
                }
                else if (PIDPOS == 2){
                    PID.flyWheel.i = prevI;
                    PID.flyWheel.d *= 1.0-diff * moveDir;
                    std::cout << "\nDA\n";
                }
                else{
                    PID.flyWheel.d = prevD;
                    PID.flyWheel.p *= 1.0- diff * moveDir;
                    std::cout << "\nPA\n";
                }

                PIDPOS++;
                if (PIDPOS > 3){
                    PIDPOS = 1;
                }
                
            }
            else{
                std::cout << "good\n";
                double diff = (bestPVal - PVal) / bestPVal;
                bestPVal = PVal;

                if (PIDPOS == 1){
                    prevP = PID.flyWheel.p;
                    PID.flyWheel.p *= 1.0+diff*moveDir;
                    std::cout << "PA\n";
                }
                else if (PIDPOS == 2){
                    prevI = PID.flyWheel.i;
                    PID.flyWheel.i *= 1.0+diff*moveDir;
                    std::cout << "IA\n";
                }
                else{
                    prevD = PID.flyWheel.d;
                    PID.flyWheel.d *= 1.0+diff*moveDir;
                    std::cout << "DA\n";
                }

            }
        }
        
        std::cout << "P: " << PID.flyWheel.p << "\n" << "I: " << PID.flyWheel.i << "\n" << "D: " << PID.flyWheel.d << "\n";
        std::cout << "pP: " << prevP << "\n" << "pI: " << prevI << "\n" << "pD: " << prevD << "\n";

        delay(10000);
    }
}

double universalTuner(void){
    return 0;
}

void universalTuner(int length, Motor *motors[length], double goal, double &currentpos(void), PID_t PID, int timeToTest, int delayTiming, bool doublePID){
    for (int i = 0; i<length; i++){
        motors[i]->set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    }

    while (1){
        static bool baseRan = false;
        static double blVal = 0;
        double pVal = 0;
        int startTime = millis();
        bool prevGood[6] = {1,1,1, 1,1,1};
        double moveDir[6][2] = {{1,1},{1,1},{1,1}, {1,1},{1,1},{1,1}};
        bool add[6] = {1,1,1,1,1,1};
        int adjustSwitch = 0;
        while (millis() - startTime < timeToTest){
            double prop = currentpos() - goal;
            static double prevProp = prop;
            static double integ = 0;
            double deriv = prop - prevProp;

            double RUNPOWER = PID.p * prop + PID.i * integ + PID.d * deriv;

            integ += prop;
            prevProp = prop;

            if (RUNPOWER > 127){
                RUNPOWER = 127;
            }
            if (RUNPOWER < -127){
                RUNPOWER = -127;
            }
            for (int i = 0; i<length; i++){
                motors[i]->move(RUNPOWER);
            }

            pVal = fabs(RUNPOWER) + fabs(deriv);
            delay(delayTiming);
        }
        if (baseRan){
            double diff = (blVal - pVal) / blVal;
            if (blVal > pVal){
                std::cout << "\n\nPVal: " << pVal << "\nBVal: " << blVal << "Success!\nAdjusting ";
                if (adjustSwitch == 0){
                    std::cout << "P\n";
                }
                else if (adjustSwitch == 1){
                    std::cout << "I\n";
                }
                else{
                    std::cout << "D\n";
                }
                std::cout << "P: " << PID.p << "\nI: " << PID.d << "\nD: " << PID.d << "\n";
                prevGood[adjustSwitch] = 1;
                blVal = pVal;
            }
            else{
                std::cout << "\n\nPVal: " << pVal << "\nBVal: " << blVal << "Fail\nAdjusting ";
                if (adjustSwitch == 0){
                    std::cout << "P\n";
                }
                else if (adjustSwitch == 1){
                    std::cout << "I\n";
                }
                else{
                    std::cout << "D\n";
                }
                std::cout << "P: " << PID.p << "\nI: " << PID.d << "\nD: " << PID.d << "\n";
                if (!prevGood[adjustSwitch]){
                    moveDir[adjustSwitch][add[adjustSwitch]] *=.5;
                    add[adjustSwitch] = !add[adjustSwitch];
                }
                prevGood[adjustSwitch] = 0;
            }

            if (adjustSwitch == 0){
                PID.p *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }
            else if (adjustSwitch == 1){
                PID.i *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }
            else{
                PID.d *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }

            if (blVal < pVal){
                adjustSwitch++;
            }
            if (adjustSwitch == 3){
                adjustSwitch = 0;
            }

        }
        else{
            blVal = pVal;
            baseRan = true;
        }
    }
}