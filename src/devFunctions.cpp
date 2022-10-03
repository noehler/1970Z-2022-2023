#include "main.h"
#include "odometry.h"
#include "pros/rtos.h"
#include "robotConfig.h"

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
  

  /*lv_obj_t * myLabel[20];
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
  }*/
  
  lv_obj_t * rpm = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(rpm, "No value Assigned yet");
  lv_obj_align(rpm, NULL, LV_ALIGN_IN_LEFT_MID, 280, 50);
  diff1.set_brake_mode(E_MOTOR_BRAKE_COAST);
  diff2.set_brake_mode(E_MOTOR_BRAKE_COAST);
  
  while(devPossible){
    static int flySpeed = 0;

    if (master.get_digital(DIGITAL_UP) && flySpeed < 127){
      flySpeed += 1;
    }
    if (master.get_digital(DIGITAL_DOWN) && flySpeed > -127){
      flySpeed -= 1;
    }

    flyWheel1 = flySpeed;
    flyWheel2 = flySpeed;

    char buffer[50];
    sprintf(buffer, "RPM : %d", flySpeed);
    lv_label_set_text(rpm, buffer);
    delay(20);

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
			runLoop = false;
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

void logVals(char name[50],double value){
  static int i = 0;
  if (name == "reset"){
    i = 0;
  }
  else{
    outVals[i] = value;
    sprintf(outNames[i], "%s", name);
    i++;
  }
}

void setAngle(objectType object, int degree){
  if (object == base){

  }
  else{

  }
}

