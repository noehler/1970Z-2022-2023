#include "main.h"
#include "pros/misc.hpp"
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

void logValue(std::string name, double value, int position){
    char buffer[name.length() + 1];
    strcpy(outNames[position], name.c_str()); 
    outVals[position] = value;
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

void outValsSDCard(void){
    if (usd::is_installed()){
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
}

 