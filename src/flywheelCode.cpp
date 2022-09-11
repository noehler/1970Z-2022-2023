#include "main.h"
#include "odometry.h"
#include "robotConfig.h"
#define DEG2RAD M_PI/180

double goalSpeed = 0;
float wheelRad = 2.5;
float gearRatio = 1;
bool autoControl = true;

double angularVelocityCalc(void){
  double attackSpeed = goalSpeed/wheelRad/DEG2RAD *gearRatio;
  std::cout << "W: " << attackSpeed << "Input: " << goalSpeed;
  return attackSpeed;
}

void turretControl(void){
  while(1){
    robotGoal.angleBetweenHorREL = (inertial.get_rotation() - robotGoal.angleBetweenHorABS);
    double spd = pow(fabs(robotGoal.angleBetweenHorREL-(float(turretAngle.get_position())/100/259*12)), 1.0/3)*15;
    if (robotGoal.angleBetweenHorREL-(float(turretAngle.get_position())/100/259*12)<0){
      spd *= -1;
    }
    //std::cout << "\nSpeed" <<spd << ", RelA:" << robotGoal.angleBetweenHorREL-(float(turretAngle.get_position())/100/259*12) << ", bA:" << inertial.get_rotation();
    diff1 = spd;
    diff2 = -spd;
    delay(20);
  }
  if (autoControl){
    double topSpeed = 600 *gearRatio;
    double rotNeeded = angularVelocityCalc()/topSpeed;

    flyWheel1 = rotNeeded;
    flyWheel2 = rotNeeded;

  }
  else{
    int intakeSPD;
		if (master.get_digital(DIGITAL_R2)){
			intakeSPD = -127;
		}
		else if (master.get_digital(DIGITAL_R1)){
			intakeSPD = 127;
		}
		else{
			intakeSPD = 0;
		}

		int turrSPD;
		if (master.get_digital(DIGITAL_A)){
			turrSPD = 50;
		}
		else if (master.get_digital(DIGITAL_B)){
			turrSPD = -50;
		}
		else{
			turrSPD = 0;
		}

		int d1SPD = intakeSPD + turrSPD;
		int d2SPD = intakeSPD - turrSPD;

		if (d1SPD > 127 || d2SPD > 127){
			d1SPD -= abs(turrSPD);
			d2SPD -= abs(turrSPD);
		}
		else if (d1SPD < -127 || d2SPD < -127){
			d1SPD += abs(turrSPD);
			d2SPD += abs(turrSPD);
		}


		diff1 = d1SPD;
		diff2 = d2SPD;
  }
  
}

//defining constants
float g = 386.08858267717;
double a = -pow(g,2)*.25;

//means iterative time based turret rotation calculator
void singSameOldSongTimeTurretTwister(void){
  //define quartic equation terms
  double c = pow(robot.xVelocity, 2) + pow(robot.yVelocity, 2) - robotGoal.dz * g;
  double d = -2 * robotGoal.dx * robot.xVelocity - 2 * robotGoal.dy * robot.yVelocity;
  double e = pow(robotGoal.dx, 2) + pow(robotGoal.dy, 2) - pow(robotGoal.dz, 2);
  double D = 1000000000000;
  double T = 0.1;

  bool close_enough = false;
  while (close_enough != true){
    if (D > 10000){
      T += 0.1;
    }  
    else if (D > 1){
      T += 0.001;
    }
    else{
      close_enough = true;
    }
    D = a * pow(T, 4) + c * pow(T, 2) + d * T + e;
  }
  
  double P1 = robotGoal.dy - robot.yVelocity * T;
  double P2 = robotGoal.dx - robot.xVelocity * T;
  double Tar_ang = atan(P1 / P2);
  double P3 = cos(Tar_ang) * 0.707106781187 * T;
  double V_disk = P2 / P3;

  //outputting calculated values
  robotGoal.angleBetweenHorABS = Tar_ang *180/M_PI;
  goalSpeed = V_disk;
  //std::cout << "\nAngle:" << robotGoal.angleBetweenHorABS;
}
