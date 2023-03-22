#ifndef __DEVFUNCS_H__
#define __DEVFUNCS_H__
#include "api.h"
extern void tuneSensors(void);
class PID_t{
    public:
        double p, i, d, p2, i2,d2;
};
#endif