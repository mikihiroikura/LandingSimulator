#ifndef _CONTROLROBOT_H
#define _CONTROLROBOT_H

#include "main.h"

extern dReal Tau[LEG_NUM];
extern MyObject piston[LEG_NUM];
extern dReal F[LEG_NUM][3];

extern void calcSLS(SIM *sim);
extern void initSLS();

#endif // !_CONTROLROBOT_H

