#include "main.h"
#include "odometry.h"
#include "pros/rtos.h"
#include "robotConfig.h"
#include <cstring>

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


FILE *usd_file_write;
char filename[20];
int fileNum;
#include <fstream>

void startRecord(void){
  for (int i = 1; ; i++)
  {
    sprintf(filename, "/usd/pos_%d.txt", i);
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
      break;
    }
  }
}

void outPosSDCARD(void){
  static int prevPosX = robot.xpos;
  static int prevPosY = robot.ypos;
  static bool start = true;
  if (start == true){
    startRecord();
    start = false;
  }
  if (fabs(robot.ypos - prevPosY) > .1 || fabs(robot.xpos - prevPosX) > .1){
    usd_file_write = fopen(filename, "a");
    char buffer[50];
    sprintf(buffer,"\n(%.2f, %.2f)", robot.xpos, robot.ypos);
    fputs(buffer, usd_file_write);
    fclose(usd_file_write);

    prevPosX = robot.xpos;
    prevPosY = robot.ypos;
  }
}

void outValsSDCard(void){
  char buffer[20];
  sprintf(buffer, "/usd/vals_%d", fileNum);
  usd_file_write = fopen(buffer, "a");
  for (int i = 0; i < 20; i++){
    fprintf(usd_file_write,"%s: %f", outNames[i], outVals[i]);
  }
  fclose(usd_file_write);
}
