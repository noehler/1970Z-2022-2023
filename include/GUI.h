#ifndef __GUI_H__
#define __GUI_H__

#ifndef _PROS_MAIN_H_
#include "api.h"
#endif

using namespace pros;

class GUI_t{
    private:
        bool devMode = false;
        bool autonChecker = false;
    public:
        void enabledLoop(void){
            while(!competition::is_disabled()){

            }
        }
        void autonSelectorAndCheck(void){
            while(competition::is_disabled()){

            }
        }

};

extern void autonSelectCheck_Wrapper(void* GUI);
extern void enabledLoop_Wrapper(void* GUI);
#endif