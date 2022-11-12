#ifndef __PIDTUNERS_H__
#define __PIDTUNERS_H__

class PID_t{
    public:
        double p, i, d, p2, i2,d2;
};
class tunedSystems_t{
    public:
        PID_t driveFR, driveSS, turret, flyWheel;
};

extern tunedSystems_t PID;

extern void PIDTunnerDrive(void);
extern void PIDTunnerTurret(void);
extern void PIDTunnerFly(void);
#endif