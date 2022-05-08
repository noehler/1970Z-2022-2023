#include "main.h"

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

void trackNums(void){
  velocity[currPos] = flyWheel1.get_actual_velocity() * 5;
  timeV[currPos] = millis();

  currPos++;
  if (currPos > 999){
    currPos = 0;
    graphVelTime();
  }
}
