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
char errorMessage[20];
double discMults[3][2] = {{1,0}, {1,0}, {1,0}};
void sidecarGUI(bool showError){
    static bool prevShow = 0;
    static int multChange[2] = {0,1};
    
    //update variables
    if (sidecar.get_digital(DIGITAL_B)){
        multChange[0] = 1;
    }
    else if (sidecar.get_digital(DIGITAL_A)){
        multChange[0] = 2;
    }
    else if (sidecar.get_digital(DIGITAL_X)){
        multChange[0] = 3;
    }
    discMults[multChange[0]-1][0] += double(sidecar.get_analog(ANALOG_RIGHT_Y))/2048;

    if (discMults[0][0] != discMults[0][1] || discMults[1][0] != discMults[1][1] || discMults[2][0] != discMults[2][1] || multChange[0] != multChange[1]){//detects change so only sends data to controller when has to
        //setting all variables to current
        discMults[0][1] = discMults[0][0];
        discMults[1][1] = discMults[1][0];
        discMults[2][1] = discMults[2][0];
        multChange[1] = multChange[0];

        //indicator of what variables are being changed
        char changeOut[10];
        sprintf(changeOut," Change %d",multChange[0]);

        //clear and print to screen
        delay(50);
        sidecar.clear();
        delay(50);
        sidecar.print(0,0,"%.2f | %s", discMults[0][0], errorMessage);
        delay(50);
        sidecar.print(1,0,"%.2f | %s", discMults[1][0], errorMessage);
        delay(50);
        sidecar.print(2,0,"%.2f | %s", discMults[2][0], changeOut);
    }
}

//determines is last warning pushed should still be shown or cleared
int waitTime = 0;
int lastWarnTime = 0;
bool showing = true;
void warnScreenUpdate(void){
    if (sidecar.is_connected()){//only run if second controller is connected
        
        if (c::millis()-lastWarnTime < waitTime && millis() > waitTime){//if should show
            if (updateScreen[1]){
                showing = true;
                updateScreen[1] = false;
            }
            delay(50);
            sidecar.rumble(".-");//rumble when showing
        }
        else{
            if (showing){//remove warning
                showing = false;
            }
        }
        sidecarGUI(showing);//update screen
    }
}

//variable to safely send a warning message
void warn(std::string message, int messageShowTime){
    lastWarnTime = millis();
    waitTime = messageShowTime;
    strcpy(errorMessage, message.c_str());
    updateScreen[1] = true;
}

//outputting to SD card in seperate thread so if there is a corrupted card it will not end what the robot is actually doing
void outValsSDCard(void){
    //while(1){
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
        //warnScreenUpdate();//update second controller screen
        //delay(40);
    //}
}
