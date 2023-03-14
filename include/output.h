#ifndef __SDLOGGING_H__
#define __SDLOGGING_H__

#include <string>
extern void logValue(std::string name, double value, int position);
extern void outValsSDCard(void);
extern void warn(std::string message, int messageShowTime = 5000);

#endif