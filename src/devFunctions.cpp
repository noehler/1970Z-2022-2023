#include "robotConfig.h"

double getNum(std::string Output){
  std::string tempDist;
  double realNum;
  while (1){
    std::cout << "\n" << Output;
    std::cin >> tempDist;

    bool notValid = false;
    try{
      realNum = std::stol(tempDist);
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
    homeGoal.xpos = i * 60 /479;

    angleVertBetween();

    double attackLow = sqrt((acceleration * robotGoal.distBetweenH) /
                            fabs(2*tan(robotGoal.angleBetweenV - lowAngle)))         * cos(robotGoal.angleBetweenV) / cos(lowAngle);

    attackLow = robotGoal.distBetweenH*60 * fabs(robotGoal.distBetweenH*tan(robotGoal.angleBetweenV))/3;

    finalDone[i] = attackLow;
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
  double motR = (cos(pow(flyWheel1.get_temperature(), sqrt(turrYRot1.get_temperature()))/(inertial.get_accel().x+1)) + sin(pow(flyWheel4.get_temperature(), sqrt(turrYRot2.get_temperature()))/(inertial.get_accel().y+1)))/2;
  //std::cout << "\nR: " <<motR << "\n";
  return motR;
}

void numTrain(void){
  double constants[3];
  double prevConstants[3][5];
  constants[0] = 45;
  constants[1] = 1;
  constants[2] = 1;
  while (1){
    double expectedDist;
    double expectedHeight = 0;
    robot.xpos = 0;
    robot.ypos = 3;
    robot.zpos = 0;
    homeGoal.zpos = 0;
    homeGoal.ypos = 0;

    static double addAmt[3];
    addAmt[0] = 5;
    addAmt[1] = 1;
    addAmt[2] = 1;

    for (int i = 0; i< 3; i++){

      //filling empty lists
      static double pctErrorPrev[5][3];
      for (int j = 0; j < 5; j++){
        pctErrorPrev[j][i] = 10000;
        prevConstants[i][j] = constants[i];
      }
      while (1)
      {
        //generating expected values
        expectedDist = (randMotor()*35);
        homeGoal.xpos = expectedDist;
        while (expectedDist < 10){
          expectedDist = (randMotor()*35);
          delay(40);
        }

        angleVertBetween();

        master.clear();
        delay(50);
        master.print(0, 0, "Ex Dist: %f", expectedDist);
        std::cout << "\n" << expectedDist;
        delay(50);

        //calculating speed for motors to spin at
        double linear = angularVelocityCalc(0, constants[0], constants[1], constants[2]);


        double rad = 2.5/12;
        double motorSpeed = linear/rad/49;

        std::cout << "\n\nC1:" << constants[0] << "\nC2:" << constants[1] << "\nC3:" << constants[2]<< "\nSpeed:" << linear;


        //spinning motors and shooting disk;
        flyWheel1 = motorSpeed;
        flyWheel2 = motorSpeed;
        flyWheel3 = motorSpeed;
        flyWheel4 = motorSpeed;

        while((fabs(flyWheel1.get_actual_velocity()) < abs(flyWheel1.get_target_velocity()) *.9)){
					//std::cout << "\n" <<fabs(flyWheel1.get_actual_velocity()) << " : " << abs(flyWheel1.get_target_velocity())*.9;
					delay(40);
				}
				std::cout << "\n\nFire!\n\n";
				while(!shootButton.get_new_press() && !master.get_digital(E_CONTROLLER_DIGITAL_A) ){
					//std::cout << "\n" <<fabs(flyWheel1.get_actual_velocity()) << " : " << abs(flyWheel1.get_target_velocity())*.9;
					delay(40);
				}


        //getting info from user;
        std::string tempDist;
        double realNum;
        while (1){
          std::cout << "\nDist: ";
          std::cin >> tempDist;

          bool notValid = false;
          try{
            realNum = std::stol(tempDist);
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

        //calculating new value
        double pctError = fabs(realNum-expectedDist)/expectedDist;
        if (pctError < pctErrorPrev[0][i]){
          for (int j = 4; j > 0; j--){
            pctErrorPrev[j][i] = pctErrorPrev[j-1][i];
            prevConstants[i][j] = pctErrorPrev[i][j-1];
          }
          pctErrorPrev[0][i] = pctError;
          prevConstants[i][0] = constants[i];
        }
        else
        {
          addAmt[i] = addAmt[i/2];
          constants[i] = prevConstants[i][0];

        }
        constants[i] += addAmt[i];

      }
    }
  }
}
