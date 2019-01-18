#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>

#include "makerobot.h"

extern dWorldID world;
extern dSpaceID space;  // �Փˌ��o�p�X�y�[�X
extern dGeomID  ground; // �n��

extern dJointGroupID contactgroup; // �R���^�N�g�O���[�v
extern dJointID sjoint1, sjoint2;//�X���C�_�[�W���C���g
extern dJointID fixed[2];//�r���{�b�g��Body�̌Œ�

typedef struct {       // MyObject�\����
	dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	double  l, r, m;       // ����[m], ���a[m]�C����[kg]
} MyObject;

extern MyObject body, leg[2], piston1, piston2;

void makelander() {
	dMass mass;
	dReal bx = 0.1; dReal by = 0.3; dReal bz = 0.05;
	dReal x0 = 0; dReal y0 = 0; dReal z0 = 3;
	//�����̂�Body�̐ݒ�
	body.m = 0.5;
	body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, body.m, bx, by, bz);
	dBodySetPosition(body.body, x0, y0, z0);
	body.geom = dCreateBox(space, bx, by, bz);
	dGeomSetBody(body.geom, body.body);

	//�r���{�b�g1
	leg[0].m = 0.09;
	leg[0].l = 0.085;
	leg[0].r = 0.007;
	leg[0].body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, leg[0].m, 3, leg[0].r, leg[0].l);
	dBodySetPosition(leg[0].body, x0, y0 - 0.5*by + leg[0].r, z0 - 0.5*bz - 0.5*leg[0].l);
	leg[0].geom = dCreateCylinder(space, leg[0].r, leg[0].l);
	dGeomSetBody(leg[0].geom, leg[0].body);

	//�r���{�b�g�s�X�g��1
	piston1.m = 0.04;
	piston1.l = 0.105;
	piston1.r = 0.006;
	piston1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston1.m, 3, piston1.r, piston1.l);
	dBodySetPosition(piston1.body, x0, y0 - 0.5*by + leg[0].r, z0 - 0.5*bz - leg[0].l - 0.5*piston1.l);
	piston1.geom = dCreateCylinder(space, piston1.r, piston1.l);
	dGeomSetBody(piston1.geom, piston1.body);

	//leg[0]��piston1�̃X���C�_�[�W���C���g
	sjoint1 = dJointCreateSlider(world, 0);
	dJointAttach(sjoint1, leg[0].body, piston1.body);
	dJointSetSliderAxis(sjoint1, 0, 0, 1);
	dJointSetSliderParam(sjoint1, dParamLoStop, -0.08);
	dJointSetSliderParam(sjoint1, dParamHiStop, 0);

	//�r���{�b�g2
	leg[1].m = 0.09;
	leg[1].l = 0.085;
	leg[1].r = 0.007;
	leg[1].body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, leg[1].m, 3, leg[1].r, leg[1].l);
	dBodySetPosition(leg[1].body, x0, y0 + 0.5*by - leg[1].r, z0 - 0.5*bz - 0.5*leg[1].l);
	leg[1].geom = dCreateCylinder(space, leg[1].r, leg[1].l);
	dGeomSetBody(leg[1].geom, leg[1].body);

	//�r���{�b�g�s�X�g��2
	piston2.m = 0.04;
	piston2.l = 0.105;
	piston2.r = 0.006;
	piston2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston2.m, 3, piston2.r, piston2.l);
	dBodySetPosition(piston2.body, x0, y0 + 0.5*by - leg[1].r, z0 - 0.5*bz - leg[1].l - 0.5*piston2.l);
	piston2.geom = dCreateCylinder(space, piston2.r, piston2.l);
	dGeomSetBody(piston2.geom, piston2.body);

	//leg[1]��piston2�̃X���C�_�[�W���C���g
	sjoint2 = dJointCreateSlider(world, 0);
	dJointAttach(sjoint2, leg[1].body, piston2.body);
	dJointSetSliderAxis(sjoint2, 0, 0, 1);
	dJointSetSliderParam(sjoint2, dParamLoStop, -0.08);
	dJointSetSliderParam(sjoint2, dParamHiStop, 0);

	//�r���{�b�g��Body�̌Œ�
	for (size_t i = 0; i < 2; i++)
	{
		fixed[i] = dJointCreateFixed(world, 0);
		dJointAttach(fixed[i], body.body, leg[i].body);
		dJointSetFixed(fixed[i]);
	}
	/*for (size_t i = 0; i < 2; i++)
	{
		leg[i].geom = dCreateGeomTransform(space);
		dGeomTransformSetGeom(leg[i].geom, body.geom);
	}*/
}

void drawlander() {
	double size[3];
	size[0] = 0.1; size[1] = 0.3; size[2] = 0.05;
	dsSetColor(0, 0, 0);
	dsDrawBox(dBodyGetPosition(body.body), dBodyGetRotation(body.body), size);
	dsSetColor(1, 0, 0);
	dsDrawCylinder(dBodyGetPosition(leg[0].body), dBodyGetRotation(leg[0].body), leg[0].l, leg[0].r);
	dsDrawCylinder(dBodyGetPosition(leg[1].body), dBodyGetRotation(leg[1].body), leg[1].l, leg[1].r);
	dsSetColor(1, 1, 1);
	dsDrawCylinder(dBodyGetPosition(piston1.body), dBodyGetRotation(piston1.body), piston1.l, piston1.r);
	dsDrawCylinder(dBodyGetPosition(piston2.body), dBodyGetRotation(piston2.body), piston2.l, piston2.r);
}

void destroylander() {
	//�W���C���g�j��
	dJointDestroy(sjoint1);
	dJointDestroy(sjoint2);
	//�{�f�B�j��
	dBodyDestroy(body.body);
	dBodyDestroy(leg->body);
	dBodyDestroy(piston1.body);
	dBodyDestroy(piston2.body);
	//�W�I���g���j��
	dGeomDestroy(body.geom);
	dGeomDestroy(leg->geom);
	dGeomDestroy(piston1.geom);
	dGeomDestroy(piston2.geom);
}