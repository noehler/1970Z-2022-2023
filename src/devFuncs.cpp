#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "sdLogging.h"

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

double universalTunerCP(void){
    return sensing.robot.turAng;
}

void universalTuner(double goal, PID_t PIDU, int timeToTest, int delayTiming, bool doublePID, int length, void* motor){
    for (int i = 0; i<length; i++){
        ((Motor*) motor)->set_brake_mode(E_MOTOR_BRAKE_BRAKE);
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
            double prop = universalTunerCP() - goal;
            static double prevProp = prop;
            static double integ = 0;
            double deriv = prop - prevProp;

            double RUNPOWER = PIDU.p * prop + PIDU.i * integ + PIDU.d * deriv;

            integ += prop;
            prevProp = prop;

            if (RUNPOWER > 127){
                RUNPOWER = 127;
            }
            if (RUNPOWER < -127){
                RUNPOWER = -127;
            }
            for (int i = 0; i<length; i++){
                ((Motor*) motor)->move(RUNPOWER);
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
                std::cout << "P: " << PIDU.p << "\nI: " << PIDU.d << "\nD: " << PIDU.d << "\n";
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
                std::cout << "P: " << PIDU.p << "\nI: " << PIDU.i << "\nD: " << PIDU.d << "\n";
                if (!prevGood[adjustSwitch]){
                    moveDir[adjustSwitch][add[adjustSwitch]] *=.5;
                    add[adjustSwitch] = !add[adjustSwitch];
                }
                prevGood[adjustSwitch] = 0;
            }

            if (adjustSwitch == 0){
                PIDU.p *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }
            else if (adjustSwitch == 1){
                PIDU.i *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }
            else{
                PIDU.d *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
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

void turretTuner(double goal, PID_t PIDU, int timeToTest, int delayTiming, bool doublePID, int length, void* motor){
    for (int i = 0; i<length; i++){
        ((Motor*) motor)->set_brake_mode(E_MOTOR_BRAKE_BRAKE);
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
            double prop = universalTunerCP() - goal;
            static double prevProp = prop;
            static double integ = 0;
            double deriv = prop - prevProp;

            double RUNPOWER = PIDU.p * prop + PIDU.i * integ + PIDU.d * deriv;

            integ += prop;
            prevProp = prop;

            if (RUNPOWER > 127){
                RUNPOWER = 127;
            }
            if (RUNPOWER < -127){
                RUNPOWER = -127;
            }
            for (int i = 0; i<length; i++){
                ((Motor*) motor)->move(RUNPOWER);
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
                std::cout << "P: " << PIDU.p << "\nI: " << PIDU.d << "\nD: " << PIDU.d << "\n";
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
                std::cout << "P: " << PIDU.p << "\nI: " << PIDU.i << "\nD: " << PIDU.d << "\n";
                if (!prevGood[adjustSwitch]){
                    moveDir[adjustSwitch][add[adjustSwitch]] *=.5;
                    add[adjustSwitch] = !add[adjustSwitch];
                }
                prevGood[adjustSwitch] = 0;
            }

            if (adjustSwitch == 0){
                PIDU.p *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }
            else if (adjustSwitch == 1){
                PIDU.i *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
            }
            else{
                PIDU.d *= 1.0+diff*moveDir[adjustSwitch][add[adjustSwitch]];
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

class moveToInfoInternal_t{
public:
    double moveToxpos, moveToypos, targetHeading, ets, speed_limit=100, errtheta=5;
    double dist = 0;          // change of position
    double distR = 0;         // chagne of right postion
    double distL = 0;         // change of left position
    double PIDSS = 0;         // PID turning speed
    double PIDFW = 0;         // PID moveforward speed
    double PIDSpeedL = 0;     // PID leftside speed
    double PIDSpeedR = 0;     // PID rightside speed
    double prevPIDFW = 0;     // PID moveforward speed at t = -1
    double prevPIDSS = 0;     // PID turning speed at t = -1
    double PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
    double PIDSSFLAT = 0;   
    int moveToforwardToggle = 1, Stop_type = 2;
    double tolerance=5;
};

class moveToInfoExternal_t{
public:
    double moveToxpos, moveToypos, targetHeading, speed_limit=100, errtheta=5;
    int moveToforwardToggle = 1, Stop_type = 2;
    double tolerance=.5;
    bool resetMoveTo;
}move;

Motor lfD(5, E_MOTOR_GEARSET_06, false);
Motor lbD(4, E_MOTOR_GEARSET_06, false);
Motor rfD(2, E_MOTOR_GEARSET_06, true);
Motor rbD(1, E_MOTOR_GEARSET_06, true); 

void calibrateDriveTo(double goalDist){
    PID.driveFR.p = 2;
    PID.driveFR.i = .5;
    PID.driveFR.d = 1;

    PID.driveSS.p = .5;
    PID.driveSS.i = 0.001;
    PID.driveSS.d = 0.3;

    logValue("FP", PID.driveFR.p, 0);
    logValue("FI", PID.driveFR.i, 1);
    logValue("FD", PID.driveFR.d, 2);
    logValue("SP", PID.driveSS.p, 3);
    logValue("SI", PID.driveSS.i, 4);
    logValue("SD", PID.driveSS.d, 5);
    if (usd::is_installed()){
        outValsSDCard();
    }

    double FRPVAL = 0;
    double bestFRPVAL = 0;
    double SSPVAL = 0;
    double bestSSPVAL = 0;
    bool baseRun = 1;
    while (1){
        double startTime = millis();
        motorControl_t mc;
        double currentheading = sensing.robot.angle*M_PI/180;

        mc.driveDist(30, PID.driveFR.p, PID.driveFR.i, PID.driveFR.d);
        delay(2000);
        FRPVAL = millis() - startTime + lfD.get_position();
        startTime = millis();
        
        static double angleTo = 0;

        if (angleTo == 0){
            angleTo = M_PI/2;
        }
        else{
            angleTo = 0;
        }

        mc.rotateTo(angleTo, PID.driveSS.p, PID.driveSS.i, PID.driveSS.d);
        delay(2000);
        currentheading = sensing.robot.angle*M_PI/180;
        double ets = angleTo - currentheading;
        FRPVAL = millis() - startTime + ets;

        
        static int FRadjustSwitch = 0;
        static int SSadjustSwitch = 0;

        if (baseRun == 1){
            bestFRPVAL = FRPVAL;
            bestSSPVAL = SSPVAL;
        }
        else{
            double FRdiff = (bestFRPVAL - FRPVAL) / bestFRPVAL;
            //linear PID adjust
            if (FRadjustSwitch == 0){
                PID.driveFR.p *= 1.0+FRdiff;
            }
            else if (FRadjustSwitch == 1){
                PID.driveFR.i *= 1.0+FRdiff;
            }
            else{
                PID.driveFR.d *= 1.0+FRdiff;
            }

            if (bestFRPVAL < FRPVAL){
                FRadjustSwitch++;
            }
            if (FRadjustSwitch == 3){
                FRadjustSwitch = 0;
            }
            
            
            //rotational PID adjust
            double SSdiff = (bestSSPVAL - SSPVAL) / bestSSPVAL;
            //linear PID adjust
            if (SSadjustSwitch == 0){
                PID.driveSS.p *= 1.0+SSdiff;
            }
            else if (SSadjustSwitch == 1){
                PID.driveSS.i *= 1.0+SSdiff;
            }
            else{
                PID.driveSS.d *= 1.0+SSdiff;
            }

            if (bestFRPVAL < FRPVAL){
                SSadjustSwitch++;
            }
            if (SSadjustSwitch == 3){
                SSadjustSwitch = 0;
            }
            logValue("FP", PID.driveFR.p, 0);
            logValue("FI", PID.driveFR.i, 1);
            logValue("FD", PID.driveFR.d, 2);
            logValue("FP", PID.driveSS.p, 3);
            logValue("FI", PID.driveSS.i, 4);
            logValue("FD", PID.driveSS.d, 5);
            if (usd::is_installed()){
                outValsSDCard();
            }
        }
        baseRun = 0;
    }
}