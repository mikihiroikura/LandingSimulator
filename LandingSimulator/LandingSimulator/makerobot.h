#pragma once

#include "main.h"

#ifdef dDOUBLE                      // 単精度と倍精度の両方に対応する
#define dsDrawSphere dsDrawSphereD  // ためのおまじない
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

extern MyObject body, leg[LEG_NUM], piston[LEG_NUM], forcesensor[LEG_NUM];

extern void makelander();
extern void drawlander();
extern void destroylander();
extern void makeforcesensor();
extern void drawforcesensor();
extern void destroyforcesensor();
