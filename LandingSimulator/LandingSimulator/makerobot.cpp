#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>

#include "makerobot.h"

extern dWorldID world;
extern dSpaceID space;  // 衝突検出用スペース
extern dGeomID  ground; // 地面

extern dJointGroupID contactgroup; // コンタクトグループ
extern dJointID sjoint1, sjoint2;//スライダージョイント
extern dJointID fixed[2];//脚ロボットとBodyの固定

typedef struct {       // MyObject構造体
	dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
	double  l, r, m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

extern MyObject body, leg[2], piston1, piston2;

void makelander() {
	dMass mass;
	dReal bx = 0.1; dReal by = 0.3; dReal bz = 0.05;
	dReal x0 = 0; dReal y0 = 0; dReal z0 = 3;
	//直方体のBodyの設定
	body.m = 0.5;
	body.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, body.m, bx, by, bz);
	dBodySetPosition(body.body, x0, y0, z0);
	body.geom = dCreateBox(space, bx, by, bz);
	dGeomSetBody(body.geom, body.body);

	//脚ロボット1
	leg[0].m = 0.09;
	leg[0].l = 0.085;
	leg[0].r = 0.007;
	leg[0].body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, leg[0].m, 3, leg[0].r, leg[0].l);
	dBodySetPosition(leg[0].body, x0, y0 - 0.5*by + leg[0].r, z0 - 0.5*bz - 0.5*leg[0].l);
	leg[0].geom = dCreateCylinder(space, leg[0].r, leg[0].l);
	dGeomSetBody(leg[0].geom, leg[0].body);

	//脚ロボットピストン1
	piston1.m = 0.04;
	piston1.l = 0.105;
	piston1.r = 0.006;
	piston1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston1.m, 3, piston1.r, piston1.l);
	dBodySetPosition(piston1.body, x0, y0 - 0.5*by + leg[0].r, z0 - 0.5*bz - leg[0].l - 0.5*piston1.l);
	piston1.geom = dCreateCylinder(space, piston1.r, piston1.l);
	dGeomSetBody(piston1.geom, piston1.body);

	//leg[0]とpiston1のスライダージョイント
	sjoint1 = dJointCreateSlider(world, 0);
	dJointAttach(sjoint1, leg[0].body, piston1.body);
	dJointSetSliderAxis(sjoint1, 0, 0, 1);
	dJointSetSliderParam(sjoint1, dParamLoStop, -0.08);
	dJointSetSliderParam(sjoint1, dParamHiStop, 0);

	//脚ロボット2
	leg[1].m = 0.09;
	leg[1].l = 0.085;
	leg[1].r = 0.007;
	leg[1].body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, leg[1].m, 3, leg[1].r, leg[1].l);
	dBodySetPosition(leg[1].body, x0, y0 + 0.5*by - leg[1].r, z0 - 0.5*bz - 0.5*leg[1].l);
	leg[1].geom = dCreateCylinder(space, leg[1].r, leg[1].l);
	dGeomSetBody(leg[1].geom, leg[1].body);

	//脚ロボットピストン2
	piston2.m = 0.04;
	piston2.l = 0.105;
	piston2.r = 0.006;
	piston2.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston2.m, 3, piston2.r, piston2.l);
	dBodySetPosition(piston2.body, x0, y0 + 0.5*by - leg[1].r, z0 - 0.5*bz - leg[1].l - 0.5*piston2.l);
	piston2.geom = dCreateCylinder(space, piston2.r, piston2.l);
	dGeomSetBody(piston2.geom, piston2.body);

	//leg[1]とpiston2のスライダージョイント
	sjoint2 = dJointCreateSlider(world, 0);
	dJointAttach(sjoint2, leg[1].body, piston2.body);
	dJointSetSliderAxis(sjoint2, 0, 0, 1);
	dJointSetSliderParam(sjoint2, dParamLoStop, -0.08);
	dJointSetSliderParam(sjoint2, dParamHiStop, 0);

	//脚ロボットとBodyの固定
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
	//ジョイント破壊
	dJointDestroy(sjoint1);
	dJointDestroy(sjoint2);
	//ボディ破壊
	dBodyDestroy(body.body);
	dBodyDestroy(leg->body);
	dBodyDestroy(piston1.body);
	dBodyDestroy(piston2.body);
	//ジオメトリ破壊
	dGeomDestroy(body.geom);
	dGeomDestroy(leg->geom);
	dGeomDestroy(piston1.geom);
	dGeomDestroy(piston2.geom);
}