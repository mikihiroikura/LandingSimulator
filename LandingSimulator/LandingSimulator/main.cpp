#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <direct.h>

#ifdef dDOUBLE                      // �P���x�Ɣ{���x�̗����ɑΉ�����
#define dsDrawSphere dsDrawSphereD  // ���߂̂��܂��Ȃ�
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define ONE_STEP 0.01
#define SIM_CNT_MAX 6000
#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
#define FILE_PATH "data/%y%m%d_%H%M%S_logs.txt"
#define FILENAME_GRAPH1 "img_jnt_pos.png"

using namespace std;

dWorldID world;  // ���͊w�v�Z�p���[���h
dSpaceID space;  // �Փˌ��o�p�X�y�[�X
dGeomID  ground; // �n��
dJointGroupID contactgroup; // �R���^�N�g�O���[�v
dsFunctions fn;
static dJointID sjoint1, sjoint2;//�X���C�_�[�W���C���g
dJointID fixed[2];//�r���{�b�g��Body�̌Œ�

typedef struct {       // MyObject�\����
	dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	double  l, r, m;       // ����[m], ���a[m]�C����[kg]
} MyObject;

MyObject body, leg[2], piston1, piston2;
int steps;
vector<double> heights,times;
char file_name[256];

// �R�[���o�b�N�֐�
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10; // �ڐG�_���̍ő�l
	dContact contact[N];     // �ڐG�_

	// �ڐG���Ă��镨�̂̂ǂ��炩���n�ʂȂ�isGround�ɔ�0���Z�b�g
	int isGround = ((ground == o1) || (ground == o2));

	// �Փˏ��̐��� n�͏Փ˓_��
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if (isGround) {
		for (int i = 0; i < n; i++) {
			contact[i].surface.mode = dContactBounce; // �ڐG�ʂ̔�������ݒ�
			contact[i].surface.bounce = 0.8;          // �����W��(0.0����1.0)
			contact[i].surface.bounce_vel = 0.0;      // �����ɕK�v�ȍŒᑬ�x

			// �ڐG�W���C���g�̐���
			dJointID c = dJointCreateContact(world, contactgroup,
				&contact[i]);
			// �ڐG���Ă���Q�̍��̂�ڐG�W���C���g�ɂ��S��
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
				dGeomGetBody(contact[i].geom.g2));
		}
	}
}

static void simLoop(int pause) {
	dSpaceCollide(space, 0, &nearCallback);  // �Փˌ��o�֐�

	dWorldStep(world, ONE_STEP);
	dJointGroupEmpty(contactgroup); // �W���C���g�O���[�v����ɂ���

	dReal bx = 0.1; dReal by = 0.3; dReal bz = 0.05;
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

	//Body�̍��x�v��
	heights.push_back(dBodyGetPosition(body.body)[2]);
	times.push_back(steps*ONE_STEP);

	steps++;
	if (steps > SIM_CNT_MAX) { dsStop(); }

}

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
	dBodySetPosition(leg[0].body, x0, y0-0.5*by+leg[0].r, z0 - 0.5*bz - 0.5*leg[0].l);
	leg[0].geom = dCreateCylinder(space, leg[0].r, leg[0].l);
	dGeomSetBody(leg[0].geom, leg[0].body);

	//�r���{�b�g�s�X�g��1
	piston1.m = 0.04;
	piston1.l = 0.105;
	piston1.r = 0.006;
	piston1.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, piston1.m, 3, piston1.r, piston1.l);
	dBodySetPosition(piston1.body, x0, y0 - 0.5*by + leg[0].r, z0 - 0.5*bz - leg[0].l-0.5*piston1.l);
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

void start()                                  /*** �O�����@***/
{
	static float xyz[3] = { 3.0,0.0,1.0 };         // ���_�̈ʒu
	static float hpr[3] = { -180, 0, 0 };          // �����̕���
	dsSetViewpoint(xyz, hpr);                     // �J�����̐ݒ�
	steps = 0;
}

void setDrawStuff()           /*** �`��֐��̐ݒ� ***/
{
	fn.version = DS_VERSION;    // �h���[�X�^�b�t�̃o�[�W����
	fn.start = &start;        // �O���� start�֐��̃|�C���^
	fn.step = &simLoop;      // simLoop�֐��̃|�C���^
	fn.path_to_textures = "C:/ode-0.13/drawstuff/textures"; // �e�N�X�`��
}

void saveData() {
	time_t timer;
	struct tm now;
	struct tm *local;
	timer = time(NULL);
	localtime_s(&now, &timer);
	strftime(file_name, 256, FILE_PATH, &now);
	FILE *fp;
	fp = fopen(file_name, "w");
	for (size_t i = 0; i < times.size(); i++)
	{
		fprintf(fp, "%f ", times[i]);
		fprintf(fp, "%f ", heights[i]);
		fprintf(fp, "\n");
	}
	fclose(fp);
}

void saveGraph() {
	FILE *gp;
	if ((gp = _popen(GNUPLOT_PATH, "w")) == NULL) { printf("Can not find %s!", GNUPLOT_PATH);}
	else { printf("GNUPLOT activates."); }
	fprintf(gp, "pl \"%s\" us 1:2 w l\n", file_name);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	_pclose(gp);
}

int main(int argc, char **argv) {
	setDrawStuff();
	dInitODE();
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -0.098);
	dWorldSetERP(world, 0.9);          // ERP�̐ݒ�
	dWorldSetCFM(world, 1e-4);         // CFM�̐ݒ�
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	ground = dCreatePlane(space, 0, 0, 1, 0);

	makelander();

	dsSimulationLoop(argc, argv, 640, 480, &fn);
	
	saveData();
	saveGraph();

	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	return 0;
}