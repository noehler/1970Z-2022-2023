#include "main.h"
#include "odometry.h"
#include "pros/rtos.h"
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
  //cout << "Select distance to travel in inches (larger is better): \n";
  int targetDist = 74;
  //cin >> targetDist;

  //getting encoder to test
  bool completed = false;
  int encoderNum = 1;
  while(!completed){
    cout << "Encoder nicks as follows:\n\n" << "\tLeft Forward Reverse: 1\n" << "\tRight Forward Reverse: 2\n" << "\tSide Side: 3\n\n";
    cout << "Select encoder to test: \n";
    //cin >> encoderNum;
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

/*
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
}*/

bool devPossible = true;
void devMode(void){
  int startTime = millis();
  delay(50);
  master.clear();
  while(devPossible){
    //std::cout << leftEncoderFB.get_value() << "\n";

    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_B)
     && master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_Y) && millis() - startTime > 500){
			runLoop = true;
      Task my_task(startLoop);
      break;
    }
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
			runLoop = false;
			devMode();
			delay(50);
			master.clear_line(1);
			delay(50);
			master.print(1, 1, "Exiting dev mode");
			runLoop = true;
			startTime = millis();
		}
}