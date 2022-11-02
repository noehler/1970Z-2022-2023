#include "main.h"
#include "odometry.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"
#include <cstring>
#include <fstream>

double outVals[20];
char outNames[20][50];

double getNum(std::string Output){
  std::string tempDist;
  double realNum;
  while (1){
    std::cout << "\n" << Output;
    std::cin >> tempDist;

    bool notValid = false;
    try{
      realNum = std::stod(tempDist);
    }
    catch(std::invalid_argument){
      notValid = true;
    }
    if (!notValid){
      break;
    }
    else{
      std::cout << "\n\n Please input valid number";
    }
  }
  return realNum;
}

void readYRot(void){
  while (1){
    std::cout << "\nPos: " << inertial.get_heading();
    delay(39);
  }
}

void graphFunction(void){
  int xRes = 480;
  int yRes = 272;

  double lowAngle = 0;
  double acceleration = 7.726667;
  robot.xpos = 0;
  robot.ypos = 3;
  robot.zpos = 0;
  homeGoal.xpos = 10;
  homeGoal.ypos = 0;
  homeGoal.zpos = 0;

  delay(50);
  master.clear();
	delay(50);
	master.print(1, 0, "Start Calc");

  double finalDone[xRes];
  for (int i = 0; i < xRes; i++){
    double tempi = i;
    homeGoal.xpos = tempi * 60 /479;

    //angleVertBetween();

    //double attackLow = sqrt((acceleration * robotGoal.distBetweenH) /
    //                        fabs(2*tan(robotGoal.angleBetweenV - lowAngle)))         * cos(robotGoal.angleBetweenV) / cos(lowAngle);

    //attackLow = robotGoal.distBetweenH*60 * fabs(robotGoal.distBetweenH*tan(robotGoal.angleBetweenV))/3;

    //finalDone[i] = attackLow;
    //std::cout << std::to_string(robotGoal.angleBetweenV) + "\n";
    delay(20);
  }

  delay(50);
  master.clear();
	delay(50);
	master.print(1, 0, "Done Calc");

  screen::set_pen(COLOR_RED);
  for (int i = 0; i < xRes; i++){
    screen::draw_circle(i+10, finalDone[i]+10, 2);
  }

  screen::draw_line(0,10,480,10);
  screen::draw_line(10,0,10,272);

  delay(50);
  master.clear();
	delay(50);
	master.print(1, 0, "Done draw");

}

double randMotor(void){
  //std::cout << "\nX: " << 1/inertial.get_accel().x << "\nY: " << 1/inertial.get_accel().y << "\nZ: " << 1/inertial.get_accel().z;
  double motR = (cos(pow(rbD.get_temperature(), sqrt(lfD.get_temperature()))/(inertial.get_accel().x+1)) + sin(pow(rfD.get_temperature(), sqrt(lbD.get_temperature()))/(inertial.get_accel().y+1)))/2;
  //std::cout << "\nR: " <<motR << "\n";
  return motR;
}

double velocity[1000];
int timeV[1000];
int currPos = 0;

void graphVelTime(void){
  FILE* usd_file_write_v = fopen("/usd/vel.txt", "w");
  for (int i = 0; i< 1000; i++){
    fprintf(usd_file_write_v,"%f \n", velocity[i]);
  }
  fclose(usd_file_write_v);

  FILE* usd_file_write_t = fopen("/usd/time.txt", "w");
  for (int i = 0; i< 1000; i++){
    fprintf(usd_file_write_t,"%d \n", timeV[i]);
  }
  fclose(usd_file_write_t);

  lcd::print(3, "done collecting");
}

//used to get the radius experimantally to calibrate the encoders (because the wheels are never perfect)
void calcRadius(void){
  using namespace std;

  //getting target distance
  cout << "Select distance to travel in inches (larger is better): \n";
  int targetDist = 74;
  cin >> targetDist;

  //getting encoder to test
  bool completed = false;
  int encoderNum = 1;
  while(!completed){
    cout << "Encoder nicks as follows:\n\n" << "\tLeft Forward Reverse: 1\n" << "\tRight Forward Reverse: 2\n" << "\tSide Side: 3\n\n";
    cout << "Select encoder to test: \n";
    cin >> encoderNum;
    if (encoderNum == 1){
      completed = true;
    }
    else if (encoderNum == 2){
      completed = true;
    }
    else if (encoderNum == 3){
      completed = true;
    }
    else{
      cout<<"\n\nretry\n\n";
    }
  }

  //getting distance rotated and calculating value continously
  while (1){
    double degTraveled;
    if (encoderNum == 1){
      degTraveled = leftEncoderFB.get_value();
    }
    else if (encoderNum == 2){
      degTraveled = rightEncoderFB.get_value();
    }
    else if (encoderNum == 3){
      degTraveled = encoderLR.get_value();
    }
    double radius = (targetDist*180)/(degTraveled* 3.1415926535);
    cout << "\nt\tRadius: " << radius <<degTraveled;
    delay(20);
  }
}

void trackNums(void){
  velocity[currPos] = flyWheel1.get_actual_velocity() * 5;
  timeV[currPos] = millis();

  currPos++;
  if (currPos > 999){
    currPos = 0;
    graphVelTime();
  }
}

bool devPossible = true;
void devMode(void){
  int startTime = millis();
  delay(50);
  master.clear();


  lv_obj_t * myLabel[20];
  for (int i = 0; i <20; i++){
    myLabel[i] = lv_label_create(lv_scr_act(), NULL); //create label and puts it on the screen
    lv_label_set_text(myLabel[i], "No value Assigned yet"); //sets label text
    if(i <=10){
      lv_obj_align(myLabel[i], NULL, LV_ALIGN_IN_LEFT_MID, 10, i*20-100);
    }
    else{
      lv_obj_align(myLabel[i], NULL, LV_ALIGN_IN_LEFT_MID, 280, (i-10)*20-100);
    }

    //std::cout << "\n" << i << "\n";
    delay(20);
  }

  /*lv_obj_t * rpm = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(rpm, "No value Assigned yet");
  lv_obj_align(rpm, NULL, LV_ALIGN_IN_LEFT_MID, 280, 50);

  diff1.set_brake_mode(E_MOTOR_BRAKE_COAST);
  diff2.set_brake_mode(E_MOTOR_BRAKE_COAST);

  int rpmSize = 300;
  double rpmCounter[rpmSize];

  for (int j = 0; j <rpmSize; j++){
    rpmCounter[j] = 0;
  }*/
  while(devPossible){
    /*static int flySpeed = 0;

    if (master.get_digital(DIGITAL_UP) && flySpeed < 127){
      flySpeed += 1;
    }
    if (master.get_digital(DIGITAL_DOWN) && flySpeed > -127){
      flySpeed -= 1;
    }

    flyWheel1 = flySpeed;
    flyWheel2 = flySpeed;
    static double rpmAVG;

    double rpmTemp = (flyWheel1.get_actual_velocity() + flyWheel1.get_actual_velocity())/2;
    for (int j = 1; j <rpmSize; j++){
      rpmCounter[j] = rpmCounter[j-1];
    }

    rpmCounter[0] =rpmTemp;

    static int c = 0;
    if (c > 50){
      for (int j = 0;j < rpmSize; j++){
        rpmAVG += rpmCounter[j];
      }
      rpmAVG = rpmAVG/rpmSize;

      c = 0;
    }
    else{
      c++;
    }*/

    /*char buffer[200];
    sprintf(buffer, "PCT : %d, RPM: %.02f", flySpeed, rpmAVG);
    lv_label_set_text(rpm, buffer);*/

    delay(20);
    for (int i=0; i < 20; i++){
      char buffer[50];
      sprintf(buffer, "%s : %f",outNames[i] ,outVals[i]);
      lv_label_set_text(myLabel[i], buffer);
      delay(20);
    }

    liftConrol();

    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_B)
     && master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_Y) && millis() - startTime > 500){
			runLoop = true;
      Task my_task(startLoop);
      break;
    }
    delay(20);
	}
}

void devCheck(void){
  static int startTime = millis();

  if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_B)
		 && master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_Y) && millis() - startTime > 500){
			delay(50);
			master.clear_line(1);
			delay(50);
			master.print(1, 1, "Entering dev mode");
			//runLoop = false;
      std::cout << "\nDevmode\n";
			devMode();
			delay(50);
			master.clear_line(1);
			delay(50);
			master.print(1, 1, "Exiting dev mode");
			startTime = millis();
		}
}

void warn(void){
  while (!master.get_digital_new_press(DIGITAL_B)){
    delay(1000);
    master.rumble(". .");
  }
  delay(50);
  master.clear();
}

void logVals(std::string name,double value){
  static int i = 0;
  if (name == "reset"){
    i = 0;
  }
  else{
    outVals[i] = value;
    sprintf(outNames[i], "%s", name.c_str());
    i++;
  }
}


char filename[20];
int fileNum;

int startRecord(void){
  for (int i = 1; ; i++)
  {
    sprintf(filename, "/usd/pos_%d.csv", i);
    std::ifstream ifile;
    ifile.open(filename);
    if (ifile)
    {
      // file exists
    }
    else
    {
      // file does not exist
      fileNum = i;
      return i;
    }
  }
}

bool startedTracking = true;
void outPosSDCARD(void){
  static double prevPosX = robot.xpos;
  static double prevPosY = robot.ypos;
  FILE *usd_file_write;
  if (startedTracking == true){
    fileNum = startRecord();
    startedTracking = false;
  }
  if (fabs(robot.ypos - prevPosY) > 1 || fabs(robot.xpos - prevPosX) > 1){
    usd_file_write = fopen(filename, "a");
    char buffer[50];
    sprintf(buffer,"\n%.2f,%.2f,%.3f, %.3f", robot.xpos, robot.ypos, float(millis())/1000, diffFlyWheelW);
    fputs(buffer, usd_file_write);
    fclose(usd_file_write);
    prevPosX = robot.xpos;
    prevPosY = robot.ypos;
  }
}

void outValsSDCard(void){
  static bool headerMade = false;
  
  char buffer[20];
  sprintf(buffer, "/usd/vals_%d.csv", fileNum);
  FILE *usd_file_write = fopen(buffer, "a");
  if (!headerMade){
    for (int i = 0; i < 20; i++){
      if (outVals[i] != 420.69){
        fprintf(usd_file_write,"%s", outNames[i]);
      }
      if (outVals[i+1] != 420.69){
        fprintf(usd_file_write,",");
      }
      else{
        break;
      }
    }
      fprintf(usd_file_write,"\n");
      headerMade = true;
  }
  for (int i = 0; i < 20; i++){
    if (outVals[i] != 420.69){
      fprintf(usd_file_write,"%f", outVals[i]);
    }
    if (outVals[i+1] != 420.69){
      fprintf(usd_file_write,",");
    }
    else{
      break;
    }
  }
  fprintf(usd_file_write,"\n");
  fclose(usd_file_write);
}

tunedSystems_t PID;
double PIDTunner(double input, double tolerance, double sensor1tar,double *sensor1,double sensor1weight,double sensor2tar,double *sensor2, double sensor2weight){
  static double P = 1;
  static double I = 1;
  static double D = 1;
  static double OutPut = 0;
  static double scoreTB = 100000000000000;
  static double prevOutPut = 0;
  static int direction = 1;
  OutPut =(P*input + I*prevOutPut*.01 + D*(OutPut - prevOutPut)/.01);
  prevOutPut = OutPut;
  //vary, depends on sensor type > linear(Drivetrain fwd&rev), looping(turret rotation, drivertrain heading)
  double curscore = fabs(sensor1tar-*sensor1)*sensor1weight+fabs(sensor2tar-*sensor2)*sensor2weight; //general
  if (curscore-20<scoreTB){//overshooting detection, exit case
    //decent logic: step back
    direction = direction*-1;
  } else if (fabs(input) < tolerance ){//target reached, update score, progress case
    if (curscore < scoreTB){
      scoreTB=curscore;
      //decent logic: progree in same direction
    }
  }

  return OutPut;
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
void PIDTunnerDrive (){
  static moveToInfoInternal_t moveI;
  //
  /*function to find best PID constant, modeling movement of robot as function
  of input of PID and Stop condition and output of time consumption and
  drifting. By keep testing performence of the function by letting robot move
  forward 48 inches, find the best intput.

  consist of four part:
  PID funciton with variable PID and Stop condition
  panalty function sum up weight of time consumption and weight of something
  else i.e drifting, amount overshot, amount tipping etc...
  step down function
  data logging function to resume progress after changee battery.*/

  // initalizationstatic 
  double IPIDSS = 0;
  static double previousets = 0;
  static double previouset = 0;
  double penalty_value = 0;
  double base_line_value = 100000;
  int PorIorDor = 0;
  int stepdirection = 1;
  bool finished = false;
  bool arrived = false;
  bool revd = false;
  double steps[10] = {0.21,  0.16,  0.11,  0.071, 0.051, 0.031, 0.021, 0.016, 0.011, 0.0071};
  double PIDSSS[6] = {1, 0, 3, 3, .1, .3}; //current best guess
  int StepSizes[6] = {1, 1, 1, 1, 1, 1};
  bool evenLoop = false;
  // main loop
  while (finished == false) {
    /*if (pros::battery::get_capacity() <= 40) { //check battery
      warn();
      return;
    }*/
    if (evenLoop){
      move.moveToxpos = 0;
      move.moveToypos = 0;
    }else{
      move.moveToxpos = 0;
      move.moveToypos = 48;
    }
    evenLoop = !evenLoop;

    penalty_value = 0;
    // controller loop (motor control thread)
    while (arrived == false) {
      static double IPIDfw = 0;
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
      PID.driveFR.p = PIDSSS[0];
      PID.driveFR.i = PIDSSS[1];
      PID.driveFR.d = PIDSSS[2];
      PID.driveSS.p = PIDSSS[3];
      PID.driveSS.i = PIDSSS[4];
      PID.driveSS.d = PIDSSS[5];
      double currentheading =robot.angle/180*M_PI;
      double etx = move.moveToxpos - robot.xpos;//change of x
      double ety = move.moveToypos - robot.ypos;//change of y
      double dist = sqrt(pow(etx, 2) + pow(ety, 2));
      double et = dist * 10;
      penalty_value +=et;

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

      //std::cout <<"\nxpos"<<robot.xpos<<" y:"<<robot.ypos<<" ang:"<<robot.angle;
      //std::cout <<"\na:"<<move.ets<<" tarx:"<<move.moveToxpos<<" tary:"<<move.moveToypos;
      
      moveI.ets = moveI.ets*180/M_PI;
      IPIDSS += moveI.ets;
      IPIDfw += et;
      moveI.PIDSS = PID.driveSS.p * moveI.ets + PID.driveSS.i * IPIDSS * .01 + PID.driveSS.d * (moveI.ets - previousets);
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
        chassis.driveTrain.leftSpd = 0;
        chassis.driveTrain.rightSpd = 0;
        chassis.driveTrain.mechSpd = 0;
        lfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rfD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        lbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        rbD.set_brake_mode(E_MOTOR_BRAKE_HOLD);

        // normal exit condition
        arrived = true;
        //motor controller reset to inital value
        move.resetMoveTo = true;
        //declear normal exit
        //record current pid value
        if (startedTracking == true){
          fileNum = startRecord();
          startedTracking = false;
        }
        if (usd::is_installed()){
          char nameBuff[25];
          sprintf(nameBuff, "/usd/PIDDrive_%d.csv", fileNum);
          FILE* usd_file_write = fopen(nameBuff, "a");
          char contBuffer[50];
          sprintf(contBuffer,"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", PID.driveFR.p, PID.driveFR.i, PID.driveFR.d, PID.driveSS.p, PID.driveSS.i, PID.driveSS.d);
          fputs(contBuffer, usd_file_write);
          fclose(usd_file_write);
        }
        
      } else {
        chassis.driveTrain.leftSpd = moveI.PIDSpeedL;
        chassis.driveTrain.rightSpd = moveI.PIDSpeedR;
        chassis.driveTrain.mechSpd = 0;
      }

      if ((penalty_value - 20) > base_line_value) {
        // exit condition two, overshooting detaction
        arrived = true;
        //forced exit, update penalty value
        penalty_value = penalty_value + et * 40;
        //motor controlelr reset to inital value, declear abnormal exit
        move.resetMoveTo = true;
        revd = true;
      }
    }

    double delta_score = base_line_value - penalty_value;
    // descent logic
    if (delta_score > 0) {
      PorIorDor += 1;
      base_line_value = penalty_value;
      stepdirection = 1;
    } else if (revd) {
      StepSizes[PorIorDor] += 2;
      revd = false;
    } else {
      StepSizes[PorIorDor] -= 1;
      revd = true;
    }
    if (StepSizes[PorIorDor] > 12) {
      StepSizes[PorIorDor] = 12;
    }
    if (StepSizes[PorIorDor] <= 0) {
      StepSizes[PorIorDor] = 0;
    }
    if (PorIorDor == 6) {
      PorIorDor = 0;
    }
    stepdirection = stepdirection * -1;
    PIDSSS[PorIorDor] =
        PIDSSS[PorIorDor] + stepdirection * steps[StepSizes[PorIorDor]];
  }
}

void PIDTunnerTurret (){
  static moveToInfoInternal_t moveI;
  //
  /*function to find best PID constant, modeling movement of robot as function
  of input of PID and Stop condition and output of time consumption and
  drifting. By keep testing performence of the function by letting robot move
  forward 48 inches, find the best intput.

  consist of four part:
  PID funciton with variable PID and Stop condition
  panalty function sum up weight of time consumption and weight of something
  else i.e drifting, amount overshot, amount tipping etc...
  step down function
  data logging function to resume progress after changee battery.*/

  // initalizationstatic 
  double IPIDSS = 0;
  static double previousets = 0;
  static double previouset = 0;
  double penalty_value = 0;
  double base_line_value = 100000;
  int PorIorDor = 0;
  int stepdirection = 1;
  bool finished = false;
  bool arrived = false;
  bool revd = false;
  double steps[10] = {0.21,  0.16,  0.11,  0.071, 0.051, 0.031, 0.021, 0.016, 0.011, 0.0071};
  double PIDSSS[3] = {.8889, .236, 1.2818}; //current best guess
  int StepSizes[3] = {5, 5, 5};
  bool evenLoop = false;
  // main loop
  srand(c::millis());
  while (finished == false) {
    if (pros::battery::get_capacity() <= 40) { //check battery
      warn();
      return;
    }
    robotGoal.angleBetweenHorABS = 180 * (double(rand() % 500 + 1)/500 - .5);
    delay(50);
    master.clear();
    delay(50);
    master.print(0,1,"goalAngle: %.2f", robotGoal.angleBetweenHorABS);
    arrived = false;

    penalty_value = 0;

    PID.turret.p = PIDSSS[0];
    PID.turret.i = PIDSSS[1];
    PID.turret.d = PIDSSS[2];
    // controller loop (motor control thread)
    delay(250);
    while (arrived == false) {
      static double PIDPosition = 0;
      static double PIDVelocity = 0;
      static double previousT=0;
      static double T = 0;
      static double PIDscalar = 1.5;
      static double gyroScalar = 21.5833333;
      static double chassisScalar = 21.5833333;
      static double turPredicScalar = 21.5833333;
      static double IPIDvel = 0;
      static double previousveldiff = 0;
      static double IPIDang = 0;
      static double previousangdiff = 0;
      double angdiff = robotGoal.angleBetweenHorABS - robot.turAng;
      T = float(millis())/1000 - previousT;
      previousT+=T;
      if (angdiff > 180){
          angdiff -= 360;
      }
      else if( angdiff < -180){
          angdiff += 360;
      }
      IPIDang += angdiff;
      PIDPosition =(PID.turret.p*angdiff + PID.turret.i*IPIDang*.01 + PID.turret.d*(angdiff - previousangdiff));
      previousangdiff = angdiff;
      double veldiff = gyroScalar*T*(inertial.get_gyro_rate().z)-robot.wVelocity*chassisScalar + turPredicScalar*robot.turvelocity+PIDPosition*PIDscalar + 0.025*recoilPrevent*goalSpeed;
      IPIDvel += veldiff;
      PIDVelocity =(0.415*veldiff + 0.135*IPIDvel*.01 + 2.6*(veldiff - previousveldiff));
      previousveldiff = veldiff;
      if (fabs(angdiff)<1&&PIDPosition==0){
        IPIDang = 0;
      }
      if (fabs(veldiff)<0.1){
        IPIDvel = 0;
      }
      static double angularvels[3] = {0,0,0};
      angularvels[2] = angularvels[1];
      angularvels[1] = angularvels[0];
      angularvels[0] = fabs(inertialTurret.get_gyro_rate().z);
      penalty_value += angdiff + (angularvels[0] + angularvels[1] + angularvels[2])/3;
      
      diff1.move(PIDVelocity);
      diff2.move(-PIDVelocity);
      if (fabs(angdiff) < 3 && abs(turretEncoder.get_velocity()) < 30) {//good exit
        // normal exit condition
        arrived = true;
        //motor controller reset to inital value
        move.resetMoveTo = true;
        //declear normal exit
        //record current pid value
        if (startedTracking == true){
          fileNum = startRecord();
          startedTracking = false;
        }
        if (usd::is_installed()){
          char nameBuff[25];
          sprintf(nameBuff, "/usd/PIDTURRET_%d.csv", fileNum);
          FILE* usd_file_write = fopen(nameBuff, "a");
          char contBuffer[50];
          sprintf(contBuffer,"%.5f,%.5f,%.5f\n", PID.turret.p, PID.turret.i, PID.turret.d);
          fputs(contBuffer, usd_file_write);
          fclose(usd_file_write);
        }
        
      }

      if ((penalty_value - 20) > base_line_value) {
        // exit condition two, overshooting detaction
        arrived = true;
        //forced exit, update penalty value
        penalty_value = penalty_value + angdiff * 40;
        //motor controlelr reset to inital value, declear abnormal exit
        revd = true;
      }
      delay(20);
    }

    double delta_score = base_line_value - penalty_value;
    // descent logic
    if (delta_score > 0) {
      PorIorDor += 1;
      base_line_value = penalty_value;
      stepdirection = 1;
    } else if (revd) {
      StepSizes[PorIorDor] += 2;
      revd = false;
    } else {
      StepSizes[PorIorDor] -= 1;
      revd = true;
    }
    if (StepSizes[PorIorDor] > 9) {
      StepSizes[PorIorDor] = 9;
    }
    if (StepSizes[PorIorDor] <= 0) {
      StepSizes[PorIorDor] = 0;
    }
    if (PorIorDor == 3) {
      PorIorDor = 0;
    }
    stepdirection = stepdirection * -1;
    PIDSSS[PorIorDor] =
        PIDSSS[PorIorDor] + stepdirection * steps[StepSizes[PorIorDor]];

  }
}

void PIDTunnerFly (){
  static moveToInfoInternal_t moveI;
  //
  /*function to find best PID constant, modeling movement of robot as function
  of input of PID and Stop condition and output of time consumption and
  drifting. By keep testing performence of the function by letting robot move
  forward 48 inches, find the best intput.

  consist of four part:
  PID funciton with variable PID and Stop condition
  panalty function sum up weight of time consumption and weight of something
  else i.e drifting, amount overshot, amount tipping etc...
  step down function
  data logging function to resume progress after changee battery.*/

  // initalizationstatic 
  double IPIDSS = 0;
  static double previousets = 0;
  static double previouset = 0;
  double penalty_value = 0;
  double base_line_value = 10000000;
  int PorIorDor = 0;
  int stepdirection = 1;
  bool finished = false;
  bool arrived = false;
  bool revd = false;
  double steps[10] = {0.21,  0.16,  0.11,  0.071, 0.051, 0.031, 0.021, 0.016, 0.011, 0.0071};
  double PIDSSS[3] = {2, .005, .001}; //current best guess
  int StepSizes[3] = {1, 1, 1};
  bool evenLoop = false;
  // main loop
  srand(c::millis());
  while (finished == false) {
    if (pros::battery::get_capacity() <= 40) { //check battery
      warn();
      return;
    }
    double goalSpd = 330 * (double(rand() % 500 + 1)/500);
    delay(50);
    master.clear();
    delay(50);
    master.print(0,1,"goalSpeed: %.2f", goalSpd);
    arrived = false;

    penalty_value = 0;

    PID.flyWheel.p = PIDSSS[0];
    PID.flyWheel.i = PIDSSS[1];
    PID.flyWheel.d = PIDSSS[2];
    // controller loop (motor control thread)
    double IPIDang = 0;
    double previousangdiff = 0;
    delay(250);
    while (arrived == false) {
      double flyWVolt;
      double flyWheelW =(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity())/2;
      double diffFlyWheelW = goalSpd-flyWheelW;

      IPIDang += diffFlyWheelW;
      flyWVolt =(PID.flyWheel.p*diffFlyWheelW + IPIDang*PID.flyWheel.i + PID.flyWheel.d*(diffFlyWheelW - previousangdiff));
      previousangdiff = diffFlyWheelW;

      flyWheel1 =flyWVolt; 
      flyWheel2 =flyWVolt; 
      if (fabs(diffFlyWheelW)<1&&flyWVolt==0){
        IPIDang = 0;
      }

      penalty_value += IPIDang;
      static double accel[3] = {0,0,0};
      accel[2] = accel[1];
      accel[1] = accel[0];
      accel[0] = flyWheelW - accel[1];
      double avgAccel = (accel[0] + accel[1] + accel[2])/3;

      if (fabs(diffFlyWheelW) < 20 && fabs(avgAccel) < 50) {//good exit
        // normal exit condition
        arrived = true;
        //motor controller reset to inital value
        //declear normal exit
        //record current pid value
        if (startedTracking == true){
          fileNum = startRecord();
          startedTracking = false;
        }
        if (usd::is_installed()){
          char nameBuff[25];
          sprintf(nameBuff, "/usd/PIDTURRET_%d.csv", fileNum);
          FILE* usd_file_write = fopen(nameBuff, "a");
          char contBuffer[50];
          sprintf(contBuffer,"%.5f,%.5f,%.5f\n", PID.flyWheel.p, PID.flyWheel.i, PID.flyWheel.d);
          fputs(contBuffer, usd_file_write);
          fclose(usd_file_write);
        }
        
      }

      if ((penalty_value - 20) > base_line_value) {
        // exit condition two, overshooting detaction
        arrived = true;
        //forced exit, update penalty value
        penalty_value = penalty_value;
        //motor controlelr reset to inital value, declear abnormal exit
        revd = true;
        delay(50);
        master.clear();
        delay(50);
        master.print(0,1,"Fuckity Fuck Fuck Fuck");
        delay(500);
      }
      
      delay(50);
      master.print(1,1,"W: %.2f, A: %.2f", diffFlyWheelW, avgAccel);
      //delay(500);
    }

    double delta_score = base_line_value - penalty_value;
    // descent logic
    if (delta_score > 0) {
      PorIorDor += 1;
      base_line_value = penalty_value;
      stepdirection = 1;
    } else if (revd) {
      StepSizes[PorIorDor] += 2;
      revd = false;
    } else {
      StepSizes[PorIorDor] -= 1;
      revd = true;
    }
    if (StepSizes[PorIorDor] > 9) {
      StepSizes[PorIorDor] = 9;
    }
    if (StepSizes[PorIorDor] <= 0) {
      StepSizes[PorIorDor] = 0;
    }
    if (PorIorDor == 3) {
      PorIorDor = 0;
    }
    stepdirection = stepdirection * -1;
    PIDSSS[PorIorDor] =
        PIDSSS[PorIorDor] + stepdirection * steps[StepSizes[PorIorDor]];

  }
}

void calibrateTurretDistances(void){
  while (1){
		static int pressed = 0;
		static int spd = 0;
		if (master.get_digital(DIGITAL_UP)){
			if (pressed > 10){
				spd+=5;
			}else{
				spd+=1;
			}
			pressed+=1;
		}
		else if (master.get_digital(DIGITAL_DOWN)){
			if (pressed < -10){
				spd-=5;
			}else{
				spd-=1;
			}
			pressed-=1;
		}
		else{
			pressed = 0;
		}
		flyWheel1 = spd;
		flyWheel2 = spd;
		delay(100);
		master.clear();
		delay(50);
		master.print(0,0,"GS: %d, RPM: %f", spd, (flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity())/2);
	}
}