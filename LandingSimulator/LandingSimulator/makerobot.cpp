#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>

#include "makerobot.h"

extern dWorldID world;
extern dSpaceID space;  // 衝突検出用スペース
extern dGeomID  ground; // 地面

extern dJointGroupID contactgroup; // コンタクトグループ
extern dJointID sjoint[LEG_NUM];//スライダージョイント
extern dJointID fixed[LEG_NUM];//脚ロボットとBodyの固定

typedef struct {       // MyObject構造体
	dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
	double  l, r, m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

extern MyObject body, leg[LEG_NUM], piston[LEG_NUM];

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
	piston[0].m = 0.04;
	piston[0].l = 0.105;
	piston[0].r = 0.006;
	piston[0].body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston[0].m, 3, piston[0].r, piston[0].l);
	dBodySetPosition(piston[0].body, x0, y0 - 0.5*by + leg[0].r, z0 - 0.5*bz - leg[0].l - 0.5*piston[0].l);
	piston[0].geom = dCreateCylinder(space, piston[0].r, piston[0].l);
	dGeomSetBody(piston[0].geom, piston[0].body);

	//leg[0]とpiston[0]のスライダージョイント
	sjoint[0] = dJointCreateSlider(world, 0);
	dJointAttach(sjoint[0], leg[0].body, piston[0].body);
	dJointSetSliderAxis(sjoint[0], 0, 0, 1);
	dJointSetSliderParam(sjoint[0], dParamLoStop, -0.08);
	dJointSetSliderParam(sjoint[0], dParamHiStop, 0);

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
	piston[1].m = 0.04;
	piston[1].l = 0.105;
	piston[1].r = 0.006;
	piston[1].body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston[1].m, 3, piston[1].r, piston[1].l);
	dBodySetPosition(piston[1].body, x0, y0 + 0.5*by - leg[1].r, z0 - 0.5*bz - leg[1].l - 0.5*piston[1].l);
	piston[1].geom = dCreateCylinder(space, piston[1].r, piston[1].l);
	dGeomSetBody(piston[1].geom, piston[1].body);

	//leg[1]とpiston[1]のスライダージョイント
	sjoint[1] = dJointCreateSlider(world, 0);
	dJointAttach(sjoint[1], leg[1].body, piston[1].body);
	dJointSetSliderAxis(sjoint[1], 0, 0, 1);
	dJointSetSliderParam(sjoint[1], dParamLoStop, -0.08);
	dJointSetSliderParam(sjoint[1], dParamHiStop, 0);

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
	dsDrawCylinder(dBodyGetPosition(piston[0].body), dBodyGetRotation(piston[0].body), piston[0].l, piston[0].r);
	dsDrawCylinder(dBodyGetPosition(piston[1].body), dBodyGetRotation(piston[1].body), piston[1].l, piston[1].r);
}

void destroylander() {
	//ジョイント破壊
	dJointDestroy(sjoint[0]);
	dJointDestroy(sjoint[1]);
	//ボディ破壊
	dBodyDestroy(body.body);
	dBodyDestroy(leg->body);
	dBodyDestroy(piston[0].body);
	dBodyDestroy(piston[1].body);
	//ジオメトリ破壊
	dGeomDestroy(body.geom);
	dGeomDestroy(leg->geom);
	dGeomDestroy(piston[0].geom);
	dGeomDestroy(piston[1].geom);
}