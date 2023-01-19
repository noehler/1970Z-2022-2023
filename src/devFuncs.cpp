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
    return 0;
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
    logValue("FP", PID.driveSS.p, 3);
    logValue("FI", PID.driveSS.i, 4);
    logValue("FD", PID.driveSS.d, 5);
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

        double currentheading = sensing.robot.angle*M_PI/180;
        move.targetHeading = currentheading;
        lfD.tare_position();
        lbD.tare_position();
        rfD.tare_position();
        rbD.tare_position();
        moveToInfoInternal_t moveI;
        double IPIDSS = 0;
        double previousets = 0;
        double IPIDfw = 0;
        double previouset = 0;
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
        moveI.PIDSSFLAT = 0; 
        move.tolerance = .5;
        move.resetMoveTo = false;
        double distTraveled = 0;
        logValue("S or T", 0, 10);

        while (!move.resetMoveTo){

            currentheading = sensing.robot.angle*M_PI/180;
            /*
            function logic:
            find errors of position, turn to target if robot cannot move in a arc to
            it, than move to target in a arc. tracking center of robot is at the
            center of two tracking wheels do not recomand using this funciton with
            SpinTo() function. perferd to have a sperate thread for calculating live
            position, than just take out codes from line 41 to line 48
            */
            double dist = goalDist - distTraveled;
            double et = dist * 10;

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
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            else{
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            if (fabs(moveI.ets) < 10) {
                if (move.moveToforwardToggle == 1){
                moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }
                else{
                moveI.PIDFW = move.moveToforwardToggle * (PID.driveFR.p * et + PID.driveFR.i * IPIDfw + PID.driveFR.d * (et - previouset));
                }
            } else {
                moveI.PIDFW = 0;
            }
            moveI.PIDFW = 50;
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

            if (fabs(dist) < move.tolerance) {
                move.resetMoveTo = true;    
                lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                lfD.brake();
                rfD.brake();
                lbD.brake();
                rbD.brake();
                logValue("lfP", 0, 6);
                logValue("lbP", 0, 7);
                logValue("rfP", 0, 8);
                logValue("rbP", 0, 9);

            } else {
                lfD.move(-moveI.PIDSpeedL);
                rfD.move(-moveI.PIDSpeedR);
                lbD.move(-moveI.PIDSpeedL);
                rbD.move(-moveI.PIDSpeedR);
                logValue("lfP", lfD.get_voltage(), 6);
                logValue("lbP", lbD.get_voltage(), 7);
                logValue("rfP", rfD.get_voltage(), 8);
                logValue("rbP", rbD.get_voltage(), 9);
            }
            distTraveled += double(lfD.get_position() + lbD.get_position() + rfD.get_position() + rbD.get_position())/8 / 180 * M_PI *1.375;
            lfD.tare_position();
            lbD.tare_position();
            rfD.tare_position();
            rbD.tare_position();
            delay(20);
        }
        lfD.brake();
        rfD.brake();
        lbD.brake();
        rbD.brake();
        delay(2000);
        while (1){
            delay(200);
        }
        FRPVAL = millis() - startTime + lfD.get_position();
        startTime = millis();
        
        //rotational Section
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
        moveI.PIDSSFLAT = 0; 
        move.resetMoveTo = false;
        static double angleTo = 0;

        if (angleTo == 0){
            angleTo = M_PI/2;
        }
        else{
            angleTo = 0;
        }
        logValue("S or T", 1, 10);
        while (!move.resetMoveTo){
            /*
            function logic:
            find errors of position, turn to target if robot cannot move in a arc to
            it, than move to target in a arc. tracking center of robot is at the
            center of two tracking wheels do not recomand using this funciton with
            SpinTo() function. perferd to have a sperate thread for calculating live
            position, than just take out codes from line 41 to line 48
            */

            currentheading = sensing.robot.angle*M_PI/180;
            moveI.ets = angleTo - currentheading;
            if (moveI.ets < -M_PI) {
            moveI.ets += 2*M_PI;
            }
            if (moveI.ets > M_PI) {
            moveI.ets -= 2*M_PI;
            }
            
            moveI.ets = moveI.ets*180/M_PI;
            IPIDSS += moveI.ets;
            if (move.moveToforwardToggle == 1){
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            else{
                moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS + PID.driveSS.d * (moveI.ets - previousets);
            }
            previousets = moveI.ets;
            moveI.PIDSSFLAT = moveI.PIDSS;
            if (moveI.PIDSSFLAT >= 2 * move.speed_limit) {
                moveI.PIDSSFLAT = 2 * move.speed_limit;
            }
            if (moveI.PIDSSFLAT <= -2 * move.speed_limit) {
                moveI.PIDSSFLAT = -2 * move.speed_limit;
            }
            
            moveI.PIDSpeedR = - moveI.PIDSSFLAT;
            moveI.PIDSpeedL = moveI.PIDSSFLAT;

            if (fabs(angleTo) < move.tolerance) {
                move.resetMoveTo = true;
                move.Stop_type = 1;
                lfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                rfD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                lbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                rbD.set_brake_mode(E_MOTOR_BRAKE_COAST);
                lfD.brake();
                rfD.brake();
                lbD.brake();
                rbD.brake();
                logValue("lfP", 0, 6);
                logValue("lbP", 0, 7);
                logValue("rfP", 0, 8);
                logValue("rbP", 0, 9);

            } else {
                lfD.move(-moveI.PIDSpeedL);
                rfD.move(-moveI.PIDSpeedR);
                lbD.move(-moveI.PIDSpeedL);
                rbD.move(-moveI.PIDSpeedR);
                logValue("lfP", lfD.get_voltage(), 6);
                logValue("lbP", lbD.get_voltage(), 7);
                logValue("rfP", rfD.get_voltage(), 8);
                logValue("rbP", rbD.get_voltage(), 9);
            }
            delay(20);
        }
        lfD.brake();
        rfD.brake();
        lbD.brake();
        rbD.brake();
        delay(2000);
        currentheading = sensing.robot.angle*M_PI/180;
        moveI.ets = angleTo - currentheading;
        FRPVAL = millis() - startTime + moveI.ets;

        
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