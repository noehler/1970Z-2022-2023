#include "main.h"
#include <fstream>

using namespace pros;

//global Variables
double outVals[20];
char outNames[20][50];

//local to file
char filename[20];


int startRecord(void){
    int fileNum;
    for (int i = 1; ; i++)
    {
        sprintf(filename, "/usd/vals_%d.csv", i);
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
    static int fileNum;
    static bool headerMade = false;
  
    char buffer[20];
    if (!headerMade){
      fileNum = startRecord();
    }
    sprintf(buffer, "/usd/vals_%d.csv", fileNum);
    FILE *usd_file_write = fopen(buffer, "a");
    if (!headerMade){
        for (int i = 0; i < 20; i++){
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
    for (int i = 0; i < 20; i++){
        if (outVals[i] != 420.69){
            fprintf(usd_file_write,"%f", outVals[i]);
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

