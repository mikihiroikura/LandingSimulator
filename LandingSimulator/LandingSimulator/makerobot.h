#pragma once

#include "main.h"

#ifdef dDOUBLE                      // ’P¸“x‚Æ”{¸“x‚Ì—¼•û‚É‘Î‰‚·‚é
#define dsDrawSphere dsDrawSphereD  // ‚½‚ß‚Ì‚¨‚Ü‚¶‚È‚¢
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

void makelander();
void drawlander();
void destroylander();
