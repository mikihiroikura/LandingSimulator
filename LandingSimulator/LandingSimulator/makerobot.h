#pragma once

#include "main.h"

#ifdef dDOUBLE                      // ’P¸“x‚Æ”{¸“x‚Ì—¼•û‚É‘Î‰‚·‚é
#define dsDrawSphere dsDrawSphereD  // ‚½‚ß‚Ì‚¨‚Ü‚¶‚È‚¢
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
