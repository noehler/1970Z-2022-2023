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

void universalTuner(double goal, double &currentpos(void), PID_t PIDU, int timeToTest, int delayTiming, bool doublePID, int length, void* motor){
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
            double prop = currentpos() - goal;
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

void moveToCalibrate(double &leftSpd, double &rightSpd, double &PF,  double &IF, double &DF, double &PS,  double &IS, double &DS){
    while (1){
        while(move.resetMoveTo == false){
            static moveToInfoInternal_t moveI;
            
            static double IPIDSS = 0;
            static double previousets = 0;
            static double IPIDfw = 0;
            static double previouset = 0;
            if (move.resetMoveTo) {
                IPIDSS = 0;
                previousets = 0;
                IPIDfw = 0;
                previouset = 0;
                moveI.dist = 0;          // change of position
                moveI.distR = 0;         // chagne of right postion
                moveI.distL = 0;         // change of left position
                moveI.PIDSS = 0;         // PID turning speed
                moveI.PIDFW = 0;         // PID moveforward speed
                moveI.PIDSpeedL = 0;     // PID leftside speed
                moveI.PIDSpeedR = 0;     // PID rightside speed
                moveI.prevPIDFW = 0;     // PID moveforward speed at t = -1
                moveI.prevPIDSS = 0;     // PID turning speed at t = -1
                moveI.PIDFWFLAT = 0;     // variable used for keeping move forward speed < 100
                moveI.PIDSSFLAT = 0;     // variable used for keeping turning speed < 20
                // variable for for calculating first turning.
                move.resetMoveTo = false;
            }
            double currentheading = sensing.robot.angle/180*M_PI;
            if (currentheading == PROS_ERR_F)
            {
                // JLO - handle error and exit, we can't continue
                std::cout << "\n headingFudge";
                return;
            }
            /*
            function logic:
            find errors of position, turn to target if robot cannot move in a arc to
            it, than move to target in a arc. tracking center of robot is at the
            center of two tracking wheels do not recomand using this funciton with
            SpinTo() function. perferd to have a sperate thread for calculating live
            position, than just take out codes from line 41 to line 48
            */
            double etx = move.moveToxpos - sensing.robot.xpos;//change of x
            double ety = move.moveToypos - sensing.robot.ypos;//change of y
            double dist = sqrt(pow(etx, 2) + pow(ety, 2));
            double et = dist * 10;

            //std::cout << "\n Hi5";
            move.targetHeading = atan(ety/etx);
            if (etx<0){
                move.targetHeading +=M_PI;
            } else if(etx ==0){
                move.targetHeading = M_PI/2*(fabs(ety)/ety);
            }
            if (move.moveToforwardToggle == -1){
                move.targetHeading +=M_PI;
            }
            moveI.ets = move.targetHeading - currentheading;
            if (moveI.ets < -M_PI) {
            moveI.ets += 2*M_PI;
            }
            if (moveI.ets > M_PI) {
            moveI.ets -= 2*M_PI;
            }
            
            moveI.ets = moveI.ets*180/M_PI;
            IPIDSS += moveI.ets;
            IPIDfw += et;
            if (move.moveToforwardToggle == 1){
                moveI.PIDSS = PS * moveI.ets + IS * IPIDSS + DS * (moveI.ets - previousets);
            }
            else{
                moveI.PIDSS = PS * moveI.ets + IS * IPIDSS + DS * (moveI.ets - previousets);
            }
            if (fabs(moveI.ets) < 10) {
                if (move.moveToforwardToggle == 1){
                    moveI.PIDFW = move.moveToforwardToggle * (PF * et + IF * IPIDfw + DF * (et - previouset));
                }
                else{
                    moveI.PIDFW = move.moveToforwardToggle * (PF * et + IF * IPIDfw + DF * (et - previouset));
                }
            } else {
                moveI.PIDFW = 0;
            }
            previousets = moveI.ets;
            previouset = et;
            moveI.PIDSSFLAT = moveI.PIDSS;
            moveI.PIDFWFLAT = moveI.PIDFW;
            if (moveI.PIDFWFLAT >= move.speed_limit) {
                moveI.PIDFWFLAT = move.speed_limit;
            }
            if (moveI.PIDFWFLAT <= -move.speed_limit) {
                moveI.PIDFWFLAT = -move.speed_limit;
            }
            if (moveI.PIDSSFLAT >= 2 * move.speed_limit) {
                moveI.PIDSSFLAT = 2 * move.speed_limit;
            }
            if (moveI.PIDSSFLAT <= -2 * move.speed_limit) {
                moveI.PIDSSFLAT = -2 * move.speed_limit;
            }
            if (move.moveToforwardToggle) {
                moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
                moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;

            } else {
                moveI.PIDSpeedR = -moveI.PIDFWFLAT - moveI.PIDSSFLAT;
                moveI.PIDSpeedL = -moveI.PIDFWFLAT + moveI.PIDSSFLAT;
            }
            if (dist < move.tolerance) {
                move.resetMoveTo = true;
                if (move.Stop_type == 1) {
                    //motor stop (hold)
                    leftSpd = 0;
                    rightSpd = 0;
                }
            } else {
            
                leftSpd = -moveI.PIDSpeedL;
                rightSpd = -moveI.PIDSpeedR;
            }
        }
        
    }
}