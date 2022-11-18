#include "main.h"

void enabledLoop_Wrapper(void* GUI) {
    ((GUI_t*) GUI)->enabledLoop();
}

void autonSelectCheck_Wrapper(void* GUI) {
    ((GUI_t*) GUI)->autonSelectorAndCheck();
}