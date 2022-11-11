#include "main.h"

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
  
  diff1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  diff2.set_brake_mode(E_MOTOR_BRAKE_HOLD);

  // initalizationstatic 
  double IPIDSS = 0;
  static double previousets = 0;
  static double previouset = 0;
  double penalty_value = 0;
  double base_line_value = 100000;
  int PorIorD = 0;
  int stepdirection[6] = {1,1,1,1,1,1};
  bool finished = false;
  bool arrived = false;
  bool failout = false;
  double steps[10] = {0.21,  0.16,  0.11,  0.071, 0.051, 0.031, 0.021, 0.016, 0.011, 0.0071};
  double PIDSSS[6] = {PID.turret.p, PID.turret.i, PID.turret.d, 0.415, .00135, 2.6}; //current best guess
  double prevPIDSSS[5][6] = {{.8889, .236, 1.2818, 0.415, .00135, 2.6}, {.8889, .236, 1.2818, 0.415, .00135, 2.6}, {.8889, .236, 1.2818, 0.415, .00135, 2.6}, {.8889, .236, 1.2818, 0.415, .00135, 2.6}, {.8889, .236, 1.2818, 0.415, .00135, 2.6}};
  bool prevFails[6] = {0, 0, 0, 0, 0, 0};
  int StepSizes[3] = {5, 5, 5};
  bool evenLoop = false;
	robot.TurintAng = 0;
  // main loop
  srand(c::millis());
  while (finished == false) {
    if (pros::battery::get_capacity() <= 40) { //check battery
      warn();
      return;
    }
    if (evenLoop){
      robotGoal.angleBetweenHorABS = 0;
    }
    else{
      robotGoal.angleBetweenHorABS = 135;
    }
    evenLoop = !evenLoop;
    
    delay(50);
    master.clear();
    delay(50);
    master.print(0,1,"goalAngle: %.2f", robotGoal.angleBetweenHorABS);
    arrived = false;

    penalty_value = 0;

    PID.turret.p = PIDSSS[0];
    PID.turret.i = PIDSSS[1];
    PID.turret.d = PIDSSS[2];
    PID.turret.p2 = PIDSSS[3];
    PID.turret.i2 = PIDSSS[4];
    PID.turret.d2 = PIDSSS[5];
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
      PIDVelocity =(PID.turret.p2*veldiff + PID.turret.i2*IPIDvel + PID.turret.d2*(veldiff - previousveldiff));
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
          sprintf(contBuffer,"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", PID.turret.p, PID.turret.i, PID.turret.d, PID.turret.p2, PID.turret.i2, PID.turret.d2);
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
        failout = true;
      }
      delay(20);
    }
    diff1.brake();
    diff2.brake();

    // descent logic
    if (!failout){
      //////////////////////////////////////////std::cout << "gout\n";
      if (stepdirection[PorIorD] < 0){
        stepdirection[PorIorD] = -1;
      }
      if (stepdirection[PorIorD] > 0){
        stepdirection[PorIorD] = 1;
      }

      //std::cout << "set 1 or -1\n";
      double diff = (base_line_value - penalty_value) / base_line_value * stepdirection[PorIorD];
      base_line_value = penalty_value;
      //std::cout << "calced diff\n";
      for (int i = 3; i > 0; i-=1){
        prevPIDSSS[i][PorIorD] = prevPIDSSS[i-1][PorIorD];
        //std::cout << "set value loop\n";
      }
      prevPIDSSS[0][PorIorD] = PIDSSS[PorIorD];
      //std::cout << "moved back 1\n";
      PIDSSS[PorIorD] *= 1.0+diff;
      //std::cout << "changed val\n";
    }
    else{
      /////////////////////////////////////////////////std::cout << "fout\n";
      PIDSSS[PorIorD] = prevPIDSSS[0][PorIorD];
      //std::cout << "reset value\n";
      stepdirection[PorIorD]*=.5;
      //std::cout << "adjusted step multiplier\n";
      if (!prevFails[PorIorD]){
        stepdirection[PorIorD] = -stepdirection[PorIorD];
      }
      //std::cout << "checked to reverse dir\n";
      PorIorD +=1;
      if (PorIorD == 6) {
        PorIorD = 0;
      }
      PIDSSS[PorIorD] *=1.0 + (.09 * stepdirection[PorIorD]);
      //std::cout << "changed PorIorDor\n";
    }

    prevFails[PorIorD] = failout;
    delay(2000);

  }
}

void PIDTunnerFly (){
  flyWheel1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  flyWheel2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
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
  double base_line_value = 1000000000000;
  int PorIorD = 1;
  double stepdirection[4] = {1,1,1,1};
  bool finished = false;
  bool arrived = false;
  bool failout = false;
  bool failoutPrev[3] = {0,0,0};
  double steps[10] = {0.021,  0.016,  0.011,  0.0071, 0.0051, 0.0031, 0.0021, 0.0016, 0.0011, 0.00071};
  double PIDSSS[4] = {PID.flyWheel.p, PID.flyWheel.i, PID.flyWheel.d, PID.flyWheel.p2}; //current best guess
  double prevPIDSSS[5][4] = {{PIDSSS[0], PIDSSS[1], PIDSSS[2], PIDSSS[3]}, {PIDSSS[0], PIDSSS[1], PIDSSS[2], PIDSSS[3]}, {PIDSSS[0], PIDSSS[1], PIDSSS[2], PIDSSS[3]}, {PIDSSS[0], PIDSSS[1], PIDSSS[2], PIDSSS[3]}, {PIDSSS[0], PIDSSS[1], PIDSSS[2], PIDSSS[3]}};
  bool prevFails[4] = {0, 0, 0, 0};
  double StepSizes[4] = {1, 1, 1, 1};
  bool evenLoop = false;
  // main loop
  srand(c::millis());
  while (finished == false) {
    static int loopNum = 0;
    static int failNum = 0;
    loopNum++;
    if (pros::battery::get_capacity() <= 40) { //check battery
      //std::cout << "battBad\n";
      warn();
      return;
    }
    else{
      //std::cout << "battGood\n";
    }
    double goalSpd = 330 * (double(rand() % 500 + 1)/500);
    goalSpd = 480;
    delay(50);
    master.clear();
    delay(50);
    master.print(0,1,"pass: %d, fail: %d", loopNum, failNum);
    delay(50);
    master.print(1,0,"steps: %.3f, %.3f,", PIDSSS[0]*100, PIDSSS[1]*100);
    delay(50);
    master.print(2,0,"%.3f, %.3f", PIDSSS[2]*100, PIDSSS[3]*100);
    arrived = false;
    

    penalty_value = 0;

    PID.flyWheel.p = PIDSSS[0];
    PID.flyWheel.i = PIDSSS[1];
    PID.flyWheel.d = PIDSSS[2];
    PID.flyWheel.p2 = PIDSSS[3];
    // controller loop (motor control thread)
    double IPIDang = 0;
    double previousangdiff = 0;
    delay(250);
    double TSTime = millis();
    while (arrived == false) {
      static int testStay = -420;
      double flyWVolt;
      double flyWheelW =(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity())/2;
      double diffFlyWheelW = goalSpd-flyWheelW;

      IPIDang += diffFlyWheelW;
      double prop = PID.flyWheel.p*diffFlyWheelW;
      double integ = IPIDang*PID.flyWheel.i;
      double deriv = PID.flyWheel.d*(diffFlyWheelW - previousangdiff);
      double prop2 = PID.flyWheel.p2 * goalSpd;
      flyWVolt = 12000.0/127*(prop + integ + deriv + prop2);
      
      previousangdiff = diffFlyWheelW;
      if (flyWVolt > 12000){
        flyWVolt = 12000;
      }
      if (flyWVolt < -12000){
        flyWVolt = -12000;
      }
      
      std::cout << millis() << "," <<  flyWheelW << "," << flyWVolt/12000*127 << "," << prop  << "," << integ  << "," << deriv  << "," << prop2 << "\n";
		  flyWheel1.move_voltage(flyWVolt); 
		  flyWheel2.move_voltage(flyWVolt); 

      penalty_value += fabs(diffFlyWheelW) + fabs(deriv);

      static double accel[3] = {0,0,0};
      accel[2] = accel[1];
      accel[1] = accel[0];
      accel[0] = flyWheelW - accel[1];
      double avgAccel = (accel[0] + accel[1] + accel[2])/3;

      ////////////////////////////////////std::cout << base_line_value << "," << penalty_value << "\n";
      if (millis() - TSTime > 10000/* || flyWheelW > goalSpd*1.2*/) {
        // exit condition two, overshooting detaction
        arrived = true;
        //failNum++;
        //motor controlelr reset to inital value, declear abnormal exit
        failout = true;
        delay(50);
        master.clear();
        delay(50);
        master.print(0,1,"Oh no");
        delay(500);
      }
      
      
      delay(20);
      //master.print(1,1,"W: %.2f, A: %.2f", diffFlyWheelW, avgAccel);
    }

    flyWheel1.brake();
    flyWheel2.brake();

    // descent logic
    if (penalty_value < base_line_value){
      //////////////////////////////////////////std::cout << "gout\n";
      

      //std::cout << "set 1 or -1\n";
      double diff = (base_line_value - penalty_value) / base_line_value * stepdirection[PorIorD];
      base_line_value = penalty_value;
      //std::cout << "calced diff\n";
      for (int i = 4; i > 0; i-=1){
        prevPIDSSS[i][PorIorD] = prevPIDSSS[i-1][PorIorD];
        //std::cout << "set value loop\n";
      }
      prevPIDSSS[0][PorIorD] = PIDSSS[PorIorD];
      //std::cout << "moved back 1\n";
      PIDSSS[PorIorD] *= 1.0+diff;
      //std::cout << "changed val\n";
      prevFails[PorIorD] = false;

    }
    else{
      /////////////////////////////////////////////////std::cout << "fout\n";
      failNum++;
      PIDSSS[PorIorD] = prevPIDSSS[0][PorIorD];
      //std::cout << "reset value\n";
      stepdirection[PorIorD]*=.5;
      //std::cout << "adjusted step multiplier\n";
      if (prevFails[PorIorD]){
        stepdirection[PorIorD] = -stepdirection[PorIorD];
      }
      //std::cout << "checked to reverse dir\n";
      PorIorD +=1;
      if (PorIorD == 3) {
        PorIorD = 0;
      }
      PIDSSS[PorIorD] *=1.0 + (.09 * stepdirection[PorIorD]);
      //std::cout << "changed PorIorDor\n";
      prevFails[PorIorD] = true;
    }
    if (usd::is_installed()){
        char nameBuff[25];
        sprintf(nameBuff, "/usd/PIDTURRET_%d.csv", fileNum);
        FILE* usd_file_write = fopen(nameBuff, "a");
        char contBuffer[50];
        sprintf(contBuffer,"%.5f,%.5f,%.5f,%.5f\n", PID.flyWheel.p, PID.flyWheel.i, PID.flyWheel.d, PID.flyWheel.p2);
        fputs(contBuffer, usd_file_write);
        fclose(usd_file_write);
    }

    //std::cout << "adjusted failout\n";
    
    
    /////////////////////////////////////////////////////////////////////////////std::cout << PorIorD << "," << PIDSSS[PorIorD] - prevPIDSSS[1][PorIorD] << "," << stepdirection[PorIorD] << "\n";
    //std::cout << "checked if PorIorD is too big\n";
    delay(2000);

  }
}

void PIDTunnerFlyHold (void){
  flyWheel1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  flyWheel2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
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
  double base_line_value = 100000000;
  double stepdirection = 1;
  bool finished = false;
  bool arrived = false;
  bool failout = false;
  bool failoutPrev = 0;
  double PIDSSS = .007; //current best guess
  double prevPIDSSS[5] = {.01, .01, .01, .01, .01};
  bool prevFail = 0;
  double StepSize = 1;
  bool evenLoop = false;
  // main loop
  srand(c::millis());
  while (finished == false) {
    static int loopNum = 0;
    static int failNum = 0;
    loopNum++;
    if (pros::battery::get_capacity() <= 40) { //check battery
      //std::cout << "battBad\n";
      warn();
      return;
    }
    else{
      //std::cout << "battGood\n";
    }
    double goalSpd = 550 * (double(rand() % 500 + 1)/500);
    delay(50);
    master.clear();
    delay(50);
    master.print(0,1,"pass: %d, fail: %d", loopNum, failNum);
    delay(50);
    master.print(1,0,"steps: %.3f, g: %.2f", PIDSSS*100, goalSpd);
    arrived = false;
    

    penalty_value = 0;

    PID.flyWheel.p2 = PIDSSS;
    // controller loop (motor control thread)
    double IPIDang = 0;
    double previousangdiff = 0;
    delay(250);
    while (arrived == false || 1) {
      static int testStay = -420;
      double flyWVolt;
      double flyWheelW =(flyWheel1.get_actual_velocity() + flyWheel2.get_actual_velocity())/2;
      double diffFlyWheelW = goalSpd-flyWheelW;

      IPIDang += diffFlyWheelW;
      double prop = PID.flyWheel.p*diffFlyWheelW;
      double integ = IPIDang*PID.flyWheel.i;
      double deriv = PID.flyWheel.d*(diffFlyWheelW - previousangdiff);
      double prop2 = PID.flyWheel.p2 * goalSpd;
      flyWVolt = 12000.0/127*(prop + integ + deriv + prop2);
      
      previousangdiff = diffFlyWheelW;
      if (flyWVolt > 12000){
        flyWVolt = 12000;
      }
      if (flyWVolt < -12000){
        flyWVolt = -12000;
      }
      std::cout << millis() << "," <<  goalSpd << "," <<  flyWheelW  << "," <<  testStay << "\n";
		  flyWheel1.move_voltage(flyWVolt); 
		  flyWheel2.move_voltage(flyWVolt); 
      if (fabs(diffFlyWheelW)<1&&flyWVolt==0){
        IPIDang = 0;
      }

      if (fabs(diffFlyWheelW) < 100){
        penalty_value += fabs(diffFlyWheelW)/* + 600*(diffFlyWheelW - previousangdiff)*/;
      }
      static double accel[3] = {0,0,0};
      accel[2] = accel[1];
      accel[1] = accel[0];
      accel[0] = flyWheelW - accel[1];
      double avgAccel = (accel[0] + accel[1] + accel[2])/3;

      if (fabs(diffFlyWheelW) < 20 && fabs(deriv) < 20) {//good exit
        if (testStay == -420){
          testStay = millis();
        }
        else if (millis() - testStay < 10000){
          
        }
        else{
          // normal exit condition
          arrived = true;
          //motor controller reset to inital value
          //declear normal exit
          //record current pid value
          if (startedTracking == true){
            fileNum = startRecord();
            startedTracking = false;
          }
          
        }
        
      }
      else{
        testStay = -420;
      }

      ////////////////////////////////////std::cout << base_line_value << "," << penalty_value << "\n";
      if ((penalty_value - 200) > base_line_value/* || flyWheelW > goalSpd*1.2*/) {
        // exit condition two, overshooting detaction
        arrived = true;
        failNum++;
        //motor controlelr reset to inital value, declear abnormal exit
        failout = true;
        delay(50);
        master.clear();
        delay(50);
        master.print(0,1,"Oh no");
        delay(500);
      }
      
      
      delay(20);
      //master.print(1,1,"W: %.2f, A: %.2f", diffFlyWheelW, avgAccel);
    }

    flyWheel1.brake();
    flyWheel2.brake();

    // descent logic
    if (!failout){
      //////////////////////////////////////////std::cout << "gout\n";
      if (stepdirection < 0){
        stepdirection = -1;
      }
      if (stepdirection > 0){
        stepdirection = 1;
      }

      //std::cout << "set 1 or -1\n";
      double diff = (base_line_value - penalty_value) / base_line_value * stepdirection;
      base_line_value = penalty_value;
      //std::cout << "calced diff\n";
      for (int i = 4; i > 0; i-=1){
        prevPIDSSS[i] = prevPIDSSS[i-1];
        //std::cout << "set value loop\n";
      }
      prevPIDSSS[0] = PIDSSS;
      //std::cout << "moved back 1\n";
      PIDSSS *= 1.0+diff;
      //std::cout << "changed val\n";
    }
    else{
      /////////////////////////////////////////////////std::cout << "fout\n";
      PIDSSS = prevPIDSSS[0];
      //std::cout << "reset value\n";
      stepdirection*=.5;
      //std::cout << "adjusted step multiplier\n";
      if (!prevFail){
        stepdirection = -stepdirection;
      }
      //std::cout << "checked to reverse dir\n";
      //std::cout << "changed PorIorDor\n";
    }

    prevFail = failout;

    //std::cout << "adjusted failout\n";
    
    
    /////////////////////////////////////////////////////////////////////////////std::cout << PorIorD << "," << PIDSSS[PorIorD] - prevPIDSSS[1][PorIorD] << "," << stepdirection[PorIorD] << "\n";
    //std::cout << "checked if PorIorD is too big\n";
    delay(2000);

  }
}

