#include "Autons/autonSetup.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"
#include <fstream>


using namespace pros;

//variables for logging and outputting
char filename[25];
double outVals[40] = {420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69,
                    420.69,420.69,420.69,420.69,420.69};
char outNames[40][50];

//message output for log files
bool messageToLog = false;
char logMessageText[100];

//variable to safely fill the outNum and outName lists
void logValue(std::string name, double value, int position){
    strcpy(outNames[position], name.c_str()); 
    outVals[position] = value;
}

//function to prepare a string to be outputted to SD Card next output cycle
void logMessage(std::string message){
    sprintf(logMessageText, "%s", message.c_str());
    messageToLog = true;
}

//locates the next open file name
int startRecord(void){
    int fileNum;
    for (int i = 1; ; i++)
    {
        sprintf(filename, "/usd/logs/vals_%d.csv", i);
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

//updates screen of second controller with updated variables and warnings
void sideScreenUpdate(void){
    static double angleOff = 0;
    static double prevAngleOff = -10201200;
    static double heightOff = 0;
    static double prevHeightOff = -10201200;
    angleOff+=double(sidecar.get_analog(ANALOG_RIGHT_X))/127*2;
    heightOff+=double(sidecar.get_analog(ANALOG_LEFT_Y))/127*2;

    if (angleOff != prevAngleOff || prevHeightOff != heightOff){
        prevAngleOff = angleOff;
        prevHeightOff = heightOff;
        sensing.positionCorrection(angleOff, heightOff);
    }

}

//outputting to SD card in seperate thread so if there is a corrupted card it will not end what the robot is actually doing
void outValsSDCard(void){
    while(1){
        if (usd::is_installed() && outVals[0] != 420.69 && !competition::is_disabled()){
            //making file and initally printing names of colums on top
            static int fileNum;
            static bool headerMade = false;
        
            char buffer[25];
            if (!headerMade){
            fileNum = startRecord();
            }
            sprintf(buffer, "/usd/logs/vals_%d.csv", fileNum);
            FILE *usd_file_write = fopen(buffer, "a");
            if (!headerMade){
                for (int i = 0; i < 40; i++){
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

            //outputting data to screen and sdCard
            for (int i = 0; i < 40; i++){
                if (outVals[i] != 420.69){
                    fprintf(usd_file_write,"%f", outVals[i]);
                    if (i < 20){
                        char buffer[20];
                        sprintf(buffer, "%s: %.4f", outNames[i], outVals[i]);
                        lv_label_set_text(outLabels[i], buffer);
                    }
                }
                if (outVals[i+1] != 420.69){
                    fprintf(usd_file_write,",");
                }
                else{
                    break;
                }
                
            }
            //log new messages after all other data outputted
            if (messageToLog){
                fprintf(usd_file_write,",%s", logMessageText);
                messageToLog = false;
            }
            fprintf(usd_file_write,"\n");
            fclose(usd_file_write);
        }
        else{
            //update screen if no SD card present
            for (int i = 0; i < 20; i++){
                if (outVals[i] != 420.69){
                    char buffer[20];
                    sprintf(buffer, "%s: %.4f", outNames[i], outVals[i]);
                    lv_label_set_text(outLabels[i], buffer);
                }
            }
        }
        sideScreenUpdate();//update second controller screen
        delay(40);
    }
}
