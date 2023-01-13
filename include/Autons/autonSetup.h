#ifndef __AUTONSETUP_H__
#define __AUTONSETUP_H__

enum autonTypes_t{
    winPointFar, winPointClose, noAuton, skillsAuton
};

extern autonTypes_t autonType;
extern bool isRed;

#endif