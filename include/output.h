#ifndef __SDLOGGING_H__
#define __SDLOGGING_H__

#include <string>
// basic class to handle each individual system
class PID_t {
public:
double p, i, d, p2, i2, d2;
};
extern void logValue(std::string name, double value, int position);
extern void outValsSDCard(void);
extern void logMessage(std::string message);
#endif