#include "Autons/autonSetup.h"
#include "main.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "robotConfig.h"
#include <fstream>


using namespace pros;

//global Variables

//local to file
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

bool messageToLog = false;
char logMessageText[100];

void logValue(std::string name, double value, int position){
    strcpy(outNames[position], name.c_str()); 
    outVals[position] = value;
}

PID_t fieldPID(void){
    PID_t temp;
    return temp;
}

//function to prepare a string to be outputted to SD Card next output cycle
void logMessage(std::string message){
    sprintf(logMessageText, "%s", message.c_str());
    messageToLog = true;
}

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

char errorMessage[20];
double discMults[3][2] = {{1,0}, {1,0}, {1,0}};
void sidecarGUI(bool showError){
    static bool prevShow = 0;
    static int multChange[2] = {0,1};
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

    if (discMults[0][0] != discMults[0][1] || discMults[1][0] != discMults[1][1] || discMults[2][0] != discMults[2][1] || multChange[0] != multChange[1]){
        discMults[0][1] = discMults[0][0];
        discMults[1][1] = discMults[1][0];
        discMults[2][1] = discMults[2][0];
        multChange[1] = multChange[0];
        char changeOut[10];
        sprintf(changeOut," Change %d",multChange[0]);
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

int waitTime = 0;
int lastWarnTime = 0;
bool showing = true;
void warnScreenUpdate(void){
    if (sidecar.is_connected()){
        
        if (c::millis()-lastWarnTime < waitTime && millis() > waitTime){
            if (updateScreen[1]){
                showing = true;
                updateScreen[1] = false;
            }
            delay(50);
            sidecar.rumble(".-");
        }
        else{
            if (showing){
                showing = false;
                delay(50);
                sidecar.clear();
            }
        }
        sidecarGUI(showing);
    }
}

void warn(std::string message, int messageShowTime){
    lastWarnTime = millis();
    waitTime = messageShowTime;
    strcpy(errorMessage, message.c_str());
    updateScreen[1] = true;
}

void outValsSDCard(void){
    while(1){
        if (usd::is_installed() && outVals[0] != 420.69 && !competition::is_disabled()){
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
            if (messageToLog){
                fprintf(usd_file_write,",%s", logMessageText);
                messageToLog = false;
            }
            fprintf(usd_file_write,"\n");
            fclose(usd_file_write);
        }
        else{
            for (int i = 0; i < 20; i++){
                if (outVals[i] != 420.69){
                    char buffer[20];
                    sprintf(buffer, "%s: %.4f", outNames[i], outVals[i]);
                    lv_label_set_text(outLabels[i], buffer);
                }
            }
        }
        warnScreenUpdate();
        delay(40);
    }
}
