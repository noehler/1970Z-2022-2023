#include "main.h"
#include "motorControl.h"
#include "pros/misc.hpp"
#include "robotConfig.h"
#include "sdLogging.h"
 double distto (double x, double y){
    return sqrt(pow(x,2)+pow(y,2));
 }


void moveto(void *mc, double xTo, double yTo, double speedLimit = 100, double tolerance = 5, double errtheta = 10, int forward = 1){
    //moveto frame parameters
    ((motorControl_t*) mc)->driveType = 0;
    ((motorControl_t*) mc)->move.moveToxpos = xTo;
    ((motorControl_t*) mc)->move.moveToypos = yTo;
    ((motorControl_t*) mc)->move.speed_limit = speedLimit;
    ((motorControl_t*) mc)->move.tolerance = tolerance;
    ((motorControl_t*) mc)->move.errtheta = errtheta;
    ((motorControl_t*) mc)->move.moveToforwardToggle = forward;
}

double pickPos(double posInput, int run){
    if (run == 0){
        return posInput;
    }
    else{
        return 144 - posInput;
    }
}
void shootdisks(void *mc){
    ((motorControl_t*) mc)->intakeRunning = 0;
    sensing.robot.turretLock = false;
    delay(20);
    if (sensing.robot.magFullness!=1){
    ((motorControl_t*) mc)->discCountChoice = 2;
    } else {
    ((motorControl_t*) mc)->discCountChoice = 1;
    }
    int starttime = millis();
    ((motorControl_t*) mc)->updatedAD = false;
    while(millis() - starttime <5000){
        //time out
        if (((motorControl_t*) mc)->updatedAD && fabs(((motorControl_t*) mc)->angdiff)<3 && (fabs(((motorControl_t*) mc)->diffFlyWheelW) + fabs(((motorControl_t*) mc)->diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            std::cout<<"good turret"<<"\n";
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//second batch
    ((motorControl_t*) mc)->raiseAScore(3);
    } else {
    ((motorControl_t*) mc)->raiseAScore(1);
    }
    delay(300);
    sensing.robot.turretLock = true;
}
void intake(void *mc){
    ((motorControl_t*) mc)->intakeRunning = 1;
    sensing.robot.turretLock = true;
}
void movevoltage(void *mc, double L, double R){
    
    ((motorControl_t*) mc)->driveType = 3;
    ((motorControl_t*) mc)->rightSpd = R;
    ((motorControl_t*) mc)->leftSpd = L;
}
void rotateto(void *mc,double ang){
    
    ((motorControl_t*) mc)->driveType = 1;
    ((motorControl_t*) mc)->HeadingTarget = ang;
}
    //sensing.set_status(totalX/loops,totalY/loops,270,100, 0);
    //standard motion frame
    /*
    //turret control parameters
    sensing.robot.turretLock = true; //true -> lock, false -> unlock
    mc.raiseAScore(3 or 1 disks shoots);
    mc.discCountChoice = 1 for single, 2 for automatic detection;

    //intake control parameter
    mc.intakeRunning = 1;

    //rotateto frame prarmeters
    mc.driveType = 1;
    mc.HeadingTarget = 270; 

    */
    //starting controller threads


void skillsAutonomous(void){
    
    motorControl_t mc;
	Task drive_Task(drive_ControllerWrapper, (void*) &mc, "My Driver Controller Task");
	Task turret_Intake_Task(turretIntake_ControllerWrapper, (void*) &mc, "Intake and Turret Controller Task");
	Task fly_Task(fly_ControllerWrapper, (void*) &mc, "My Flywheel Speed Controller Task");
	Task SSOSTTT_Task(SSOSTTT_Wrapper, (void*) &sensing, "turret angle Task");
    sensing.robot.turretLock = true;
    double startTime = millis();
    sensing.set_status(37,18,270,100, 0);
    movevoltage(&mc,0,0);
    delay(50);
    //shoot preload disks
    shootdisks(&mc);

    //move to roller
    moveto(&mc, 37,-10, 100,0,10,1);
    while (sensing.robot.ypos>16){
        delay(10);
    }
    mc.driveToRoller(1500);

    //move out of roller
    moveto(&mc, 37, 40,100,5,20,-1);
    while(sensing.robot.ypos<18){
        delay(10);
    }
    
    //rotate for first triple stack
    rotateto(&mc,90);
    startTime = millis();
    while(fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3000){
        delay(10);
    }

    //collect first triple stack
    intake(&mc);
    moveto(&mc, 37, 37,100,1,5,1);
    startTime = millis();
    while(sensing.robot.ypos<35.7  && millis() - startTime <5000 && sensing.robot.magFullness <3){///////////////////// if this exits out mag fullness will be true, maybe remove it here
        // check magazine full
        // check position
        // time out
        delay(10);
    }
    movevoltage (&mc,3000,3000);
    startTime = millis();
    while(sensing.robot.ypos<48  && millis() - startTime <5000 && sensing.robot.magFullness <3){/////////////////////why are there 2 of these
        // check magazine full
        // check position
        // time out
        delay(10);
    }

    movevoltage (&mc,0,0);
    delay(1000);

    //shooting first tripple stack
    shootdisks(&mc);

    //moving back to line up with second roller
    moveto(&mc, 35.7, 35.7,100,1,5,-1);
    startTime = millis();
    while(sensing.robot.ypos > 36&&millis() - startTime <3000){
        delay(10);
    }

    //turning to roller
    moveto(&mc,0,37.5,100,1,5,1);
    startTime = millis();
    while(sensing.robot.xpos>18&&millis() - startTime <2000){
        delay(10);
    }

    //turning second roller
    mc.driveToRoller(2500);

    //backing out of roller
    moveto(&mc,37.5,37.5,100,1,20,-1);
    startTime = millis();
    while(sensing.robot.xpos < 24&&millis() - startTime <2000){
        delay(10);
    }

    //move to collect first disc from first set of boarder discs
    moveto(&mc,26.7,89,100,1,5,1);
    startTime = millis();
    intake(&mc);
    while(sensing.robot.ypos < 89&&millis() - startTime <5000){
        delay(10);
    }

    //collect first set of border discs
    moveto(&mc,48,89,100,2,20,1);
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        //robot to target dist check
        //time out
        delay(10);
    }
    
    //shoot fist set of border disccs
    movevoltage (&mc,0,0);
    delay(1000);
    shootdisks(&mc);









    delay(10000000);
    int i = 0;
    //move to lineup disk
    moveto(&mc, pickPos(43, i), pickPos(28,i),100,2,10,-1);
    startTime = millis();
    while(sensing.robot.ypos<27 &&millis() - startTime <1500){
        delay(10);
    }
    //pick up disk
    moveto(&mc, pickPos(0,i), pickPos(26,i),35,10,10,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while((sensing.robot.xpos>pickPos(24, 0) || sensing.robot.xpos<pickPos(24, 1))&&millis()-startTime<1000){
        delay(10);
    }
    //line up with the roller might need a hold on drive motor
    mc.driveType = 1;
    if(i == 0){
        mc.HeadingTarget = 180;
    }
    else{
        mc.HeadingTarget = 0;
    }
    startTime = millis();
    while (fabs(sensing.robot.angle -180)+fabs(sensing.robot.velW)>=3&&millis() - startTime <1500){ ///////////////////////////////////////////////////////////////////////////// no time limit
        delay(10);
    }

    //drive to roller
    moveto(&mc, pickPos(0,i), pickPos(26,i),100,10,5,1);
    startTime = millis();
    while((sensing.robot.xpos>pickPos(18, 0) || sensing.robot.xpos<pickPos(18, 1))&&millis()-startTime<1000){
        delay(10);
    }
    //get the roller
    mc.driveToRoller(  1500);
    mc.intakeRunning = 0;

    //out of roller
    moveto(&mc, pickPos(20,i),pickPos(26,i),50,5,10,-1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(18, 0) || sensing.robot.xpos>pickPos(18, 1))&&millis()-startTime<1000){
        //time out
        //robot pos check
        delay(10);
    }
    //start aiming and and getting position for next disk
    sensing.robot.turretLock = false;
    mc.discCountChoice = 2;
    moveto(&mc, pickPos(24,i),pickPos(49,i),100,5,90,-1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(24, 0) || sensing.robot.xpos>pickPos(24, 1))&&millis()-startTime<1000){
        // robot pos check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(15,i),pickPos(85,i),80,5,5,1);
    //finish moving and prep for shooting
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        //robot to target dist check
        //time out
        delay(10);
    }
    //shoot by condition
    startTime = millis();
    while(millis() - startTime <3000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2))/2 +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    mc.raiseAScore(3);//first batch
    delay(300);
    //reset turret, rotate to disks
    sensing.robot.turretLock = true;
    mc.driveType = 1;
    if (i == 0){
        mc.HeadingTarget = 0;
    }
    else{
        mc.HeadingTarget = 180;
    }
    startTime = millis();
    while (fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3000 &&fabs(mc.angdiff)>3 ){
        //robot heading check
        //time out
        //turret angle check before intake 
        delay(10);
    }

    //getting in position to scrape along barrier
    moveto(&mc, pickPos(28,i),pickPos(88,i),50,5,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        //robot to target dist check
        //time out
        delay(10);
    }

    //start intaking 
    mc.intakeRunning = 1;
    moveto(&mc, pickPos(48,i),pickPos(89,i),50,5,10,1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(48, 0) || sensing.robot.xpos>pickPos(48, 1)) && millis() - startTime <5000 && sensing.robot.magFullness <3){
        // check magazine full
        // check position
        // time out
        delay(10);
    }
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//second batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);
    sensing.robot.turretLock = true;

    moveto(&mc, pickPos(55.5,i),pickPos(82,i),100,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    mc.intakeRunning = 1;
    mc.driveType = 1;
    if(i == 0){
        mc.HeadingTarget = 90;
    }
    else{
        mc.HeadingTarget = 270;
    }
    while (fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <2000  ){
        //robot heading check
        //time out
        delay(10);
    }

    moveto(&mc, pickPos(55.5,i),pickPos(124,i),40,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <5000&&sensing.robot.magFullness < 3){
        //robot to target dist check
        //time out
        //check if magazine if full
        delay(10);
    }
    
    mc.driveType = 3;
    mc.rightSpd = 0;
    mc.leftSpd = 0;
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//third batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);

    sensing.robot.turretLock = true;
    moveto(&mc, pickPos(55.5,i),pickPos(150,i),100,1,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while (sensing.robot.ypos<130 && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    
    mc.driveType = 1;
    if(i ==0){
        mc.HeadingTarget = 320;
    }
    else{
        mc.HeadingTarget = 140;
    }
    startTime = millis();
    while (fabs(sensing.robot.angle -mc.HeadingTarget)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3500){
        //time out
        //heading check
        delay(10);
    }
    moveto(&mc, pickPos(88,i),pickPos(109,i),100,5,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(88,i),pickPos(109,i),40,5,10,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime < 2000 && sensing.robot.magFullness <3){
        //robot to target dist check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(70,i),pickPos(117,i),100,5,10,-1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000 &&sensing.robot.magFullness <3){
        //robot to target dist check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(88,i),pickPos(109,i),40,5,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000 && sensing.robot.magFullness <3){
        //robot to target dist check
        //time out
        delay(10);
    }
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//forth batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);
    
    if (i == 0){
        mc.driveType = 1;
        if(i == 0){
            mc.HeadingTarget = 45;
        }
        else{
            mc.HeadingTarget = 135;
        }
        while (fabs(sensing.robot.angle -mc.HeadingTarget)+fabs(sensing.robot.velW)>=3 &&millis() - startTime < 1000){
            //time out
            //heading check
            delay(10);
        }
        moveto(&mc, pickPos(107,i),pickPos(123,i),100,2,10,1);
        startTime = millis();
        while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000){
            //robot to target dist check
            //time out
            delay(10);
        }
        mc.driveType = 1;
        mc.HeadingTarget = 90;
        startTime = millis();
        while (fabs(sensing.robot.angle -mc.HeadingTarget)+fabs(sensing.robot.velW)>=3 &&millis() - startTime < 2000){
            //time out
            //heading check
            delay(10);
        }
        startTime = millis();
        moveto(&mc, 107,150, 100,5,10,1);
        while (sensing.robot.ypos<130 && millis() - startTime < 2000){
            delay(10);
        }

        mc.driveToRoller(2500);
    }
    sensing.goal.xpos = 124;
    sensing.goal.ypos = 20;
    sensing.robot.turretLock = true;

    i++;
    //move to lineup disk
    moveto(&mc, pickPos(43, i), pickPos(28,i),100,2,10,-1);
    startTime = millis();
    while(sensing.robot.ypos<27 &&millis() - startTime <1500){
        delay(10);
    }
    //line up with disk and roller might need a hold on drive motor
    mc.driveType = 1;
    if(i == 0){
        mc.HeadingTarget = 180;
    }
    else{
        mc.HeadingTarget = 0;
    }
    startTime = millis();
    while (fabs(sensing.robot.angle -180)+fabs(sensing.robot.velW)>=3&&millis() - startTime <1500){ ///////////////////////////////////////////////////////////////////////////// no time limit
        delay(10);
    }
    //pick up disk
    moveto(&mc, pickPos(0,i), pickPos(26,i),35,10,10,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while((sensing.robot.xpos>pickPos(24, 0) || sensing.robot.xpos<pickPos(24, 1))&&millis()-startTime<1000){
        delay(10);
    }
    //drive to roller
    moveto(&mc, pickPos(0,i), pickPos(26,i),100,10,5,1);
    startTime = millis();
    while((sensing.robot.xpos>pickPos(18, 0) || sensing.robot.xpos<pickPos(18, 1))&&millis()-startTime<1000){
        delay(10);
    }
    //get the roller
    mc.intakeRunning = 0;
    mc.driveToRoller(  1500);

    //out of roller
    moveto(&mc, pickPos(20,i),pickPos(26,i),50,5,10,-1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(18, 0) || sensing.robot.xpos>pickPos(18, 1))&&millis()-startTime<1000){
        //time out
        //robot pos check
        delay(10);
    }
    //start aiming and and getting position for next disk
    sensing.robot.turretLock = false;
    mc.discCountChoice = 2;
    moveto(&mc, pickPos(24,i),pickPos(49,i),100,5,90,-1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(24, 0) || sensing.robot.xpos>pickPos(24, 1))&&millis()-startTime<1000){
        // robot pos check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(13,i),pickPos(85,i),80,5,5,1);
    //finish moving and prep for shooting
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        //robot to target dist check
        //time out
        delay(10);
    }
    //shoot by condition
    startTime = millis();
    while(millis() - startTime <3000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2))/2 +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    mc.raiseAScore(3);//first batch
    delay(300);
    //reset turret, rotate to disks
    sensing.robot.turretLock = true;
    mc.driveType = 1;
    if (i == 0){
        mc.HeadingTarget = 0;
    }
    else{
        mc.HeadingTarget = 180;
    }
    startTime = millis();
    while (fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3000 &&fabs(mc.angdiff)>3 ){
        //robot heading check
        //time out
        //turret angle check before intake 
        delay(10);
    }

    //getting in position to scrape along barrier
    moveto(&mc, pickPos(28,i),pickPos(88,i),50,5,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while ((distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000)){
        //robot to target dist check
        //time out
        delay(10);
    }

    //start intaking 
    mc.intakeRunning = 1;
    moveto(&mc, pickPos(48,i),pickPos(89,i),50,5,10,1);
    startTime = millis();
    while((sensing.robot.xpos<pickPos(48, 0) || sensing.robot.xpos>pickPos(48, 1)) && millis() - startTime <5000 && sensing.robot.magFullness <3){
        // check magazine full
        // check position
        // time out
        delay(10);
    }
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//second batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);
    sensing.robot.turretLock = true;

    moveto(&mc, pickPos(55.5,i),pickPos(82,i),100,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    mc.intakeRunning = 1;
    mc.driveType = 1;
    if(i == 0){
        mc.HeadingTarget = 90;
    }
    else{
        mc.HeadingTarget = 270;
    }
    while (fabs(sensing.robot.angle)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <2000  ){
        //robot heading check
        //time out
        delay(10);
    }

    moveto(&mc, pickPos(55.5,i),pickPos(124,i),40,1,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <5000&&sensing.robot.magFullness < 3){
        //robot to target dist check
        //time out
        //check if magazine if full
        delay(10);
    }
    
    mc.driveType = 3;
    mc.rightSpd = 0;
    mc.leftSpd = 0;
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//third batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);

    sensing.robot.turretLock = true;
    moveto(&mc, pickPos(55.5,i),pickPos(150,i),100,1,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while (sensing.robot.ypos<130 && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    
    mc.driveType = 1;
    if(i ==0){
        mc.HeadingTarget = 320;
    }
    else{
        mc.HeadingTarget = 140;
    }
    startTime = millis();
    while (fabs(sensing.robot.angle -mc.HeadingTarget)+fabs(sensing.robot.velW)>=3 &&millis() - startTime <3500){
        //time out
        //heading check
        delay(10);
    }
    moveto(&mc, pickPos(88,i),pickPos(109,i),100,5,10,1);
    //finish moving and prep for shooting
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000){
        //robot to target dist check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(88,i),pickPos(109,i),40,5,10,1);
    mc.intakeRunning = 1;
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime < 2000 && sensing.robot.magFullness <3){
        //robot to target dist check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(70,i),pickPos(117,i),100,5,10,-1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000 &&sensing.robot.magFullness <3){
        //robot to target dist check
        //time out
        delay(10);
    }
    moveto(&mc, pickPos(88,i),pickPos(109,i),40,5,10,1);
    startTime = millis();
    while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <2000 && sensing.robot.magFullness <3){
        //robot to target dist check
        //time out
        delay(10);
    }
    mc.intakeRunning = 0;
    sensing.robot.turretLock = false;
    if (sensing.robot.magFullness!=1){
    mc.discCountChoice = 2;
    } else {
    mc.discCountChoice = 1;
    }
    startTime = millis();
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<3 && (fabs(mc.diffFlyWheelW) + fabs(mc.diffFlyWheelW2)) +fabs(sensing.robot.velX)+fabs(sensing.robot.velY)< 30){
            break;
            //flywheel speed check
            //turret heading check
            //robot speed check
        }
    }
    if (sensing.robot.magFullness!=1){//forth batch
    mc.raiseAScore(3);
    } else {
    mc.raiseAScore(1);
    }
    delay(300);
    
    if (i == 0){
        mc.driveType = 1;
        if(i == 0){
            mc.HeadingTarget = 45;
        }
        else{
            mc.HeadingTarget = 135;
        }
        while (fabs(sensing.robot.angle -mc.HeadingTarget)+fabs(sensing.robot.velW)>=3 &&millis() - startTime < 1000){
            //time out
            //heading check
            delay(10);
        }
        moveto(&mc, pickPos(107,i),pickPos(123,i),100,2,10,1);
        startTime = millis();
        while (distto(sensing.robot.xpos-mc.move.moveToxpos,sensing.robot.ypos-mc.move.moveToypos)>=mc.move.tolerance && millis() - startTime <3000){
            //robot to target dist check
            //time out
            delay(10);
        }
        mc.driveType = 1;
        mc.HeadingTarget = 90;
        startTime = millis();
        while (fabs(sensing.robot.angle -mc.HeadingTarget)+fabs(sensing.robot.velW)>=3 &&millis() - startTime < 2000){
            //time out
            //heading check
            delay(10);
        }
        startTime = millis();
        moveto(&mc, 107,150, 100,5,10,1);
        while (sensing.robot.ypos<130 && millis() - startTime < 2000){
            delay(10);
        }

        mc.driveToRoller(2500);
    }


    sensing.goal.xpos = 72;
    sensing.goal.ypos = 72;
    sensing.robot.turretLock = false;
    startTime = millis();
    moveto(&mc, 24,24, 100,5,5,1);
    mc.waitPosTime(3000);

    startTime = millis();
    
    mc.driveType = 3;
    mc.rightSpd = 0;
    mc.leftSpd = 0;
    while(millis() - startTime <2000){
        //time out
        if (fabs(mc.angdiff)<20){
            break;
            //expansion
        }
    }
    mc.explode();

}