#pragma once

#include "main.h"

#ifdef dDOUBLE                      // �P���x�Ɣ{���x�̗����ɑΉ�����
#define dsDrawSphere dsDrawSphereD  // ���߂̂��܂��Ȃ�
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

void makelander();
void drawlander();
void destroylander();
