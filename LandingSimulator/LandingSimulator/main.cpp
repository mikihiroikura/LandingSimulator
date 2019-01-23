#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <direct.h>

#include "makerobot.h"

#ifdef dDOUBLE                      // �P���x�Ɣ{���x�̗����ɑΉ�����
#define dsDrawSphere dsDrawSphereD  // ���߂̂��܂��Ȃ�
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
#define FILE_PATH "data/%y%m%d_%H%M%S_logs.txt"
#define FILENAME_GRAPH1 "img_jnt_pos.png"
#define FILENAME_GRAPH2 "img_leg_length.png"

using namespace std;

dWorldID world;  // ���͊w�v�Z�p���[���h
dSpaceID space;  // �Փˌ��o�p�X�y�[�X
dGeomID  ground; // �n��
dJointGroupID contactgroup; // �R���^�N�g�O���[�v
dsFunctions fn;
dJointID sjoint[LEG_NUM];//�X���C�_�[�W���C���g
dJointID fixed[LEG_NUM];//�r���{�b�g��Body�̌Œ�

typedef struct {       // MyObject�\����
	dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	double  l, r, m;       // ����[m], ���a[m]�C����[kg]
} MyObject;

MyObject body, leg[LEG_NUM], piston[LEG_NUM];
char file_name[256];

float xyz[3] = { 3.0,0.0,1.0 };         // ���_�̈ʒu
float hpr[3] = { -180, 0, 0 };          // �����̕���

SIM sim;

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

void start()                                  /*** �O�����@***/
{
	dsSetViewpoint(xyz, hpr);                     // �J�����̐ݒ�
	sim.steps = 0;
}

//�V�~�����[�V�������X�^�[�g�֐�
void restart() {
	sim.steps = 0;
	//�j��
	destroylander();
	dJointGroupDestroy(contactgroup);
	//�Đ���
	contactgroup = dJointGroupCreate(0);
	makelander();
}

//�L�[�{�[�h�̃R�}���h
static void command(int cmd)
{
	switch (cmd) {
	case 'x': xyz[0] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// x����
	case 'X': xyz[0] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -x����
	case 'y': xyz[1] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// y����
	case 'Y': xyz[1] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -y����
	case 'z': xyz[2] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// z����
	case 'Z': xyz[2] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -z����
	//case 'u':	dBodyAddForce(obj.body, 500.0, 0.0, 0.0);	break;
	case 'r':	restart();	break;
	case 'q':	dsStop();	break;
	default:printf("key missed \n"); break;
	}
}

//�n�ʂƂ̋����v�Z
void calcDist() {
	for (size_t i = 0; i < LEG_NUM; i++)
	{
		sim.leglen[i] = dJointGetSliderPosition(sjoint[i]);
		sim.legvel[i] = dJointGetSliderPositionRate(sjoint[i]);
		for (size_t j = 0; j < DIM3; j++)
		{
			sim.legpos[i][j] = dBodyGetPosition(leg[i].body)[j];
		}
		sim.Rot[i] = dBodyGetRotation(leg[i].body);
		dReal temp[3];
		dReal vec[3];
		vec[2] = leg[i].l / 2.0;
		dMultiply0(temp, sim.Rot[i], vec, 3, 3, 1);
		sim.dist[i] = sim.legpos[i][2] + temp[2];
	}
}

//�n�ʂƂ̋������狁�߂鉼�z��
void ctrlLeg() {
	//������
	if (sim.steps==0)
	{

	}
}

//�v�Z���ʂ����O�z��Ɋi�[
void copyLogData() {
	if (sim.steps<SIM_CNT_MAX)
	{
		sim.heights[sim.steps] = dBodyGetPosition(body.body)[2];
		sim.times[sim.steps] = sim.steps*ONE_STEP;
		for (size_t i = 0; i < LEG_NUM; i++)
		{
			sim.log_leglen[sim.steps][i] = sim.leglen[i];
			sim.log_legvel[sim.steps][i] = sim.legvel[i];
		}
	}
}

//ODE�V�~�����[�V�������[�v
static void simLoop(int pause) {
	//UAV�I�u�W�F�N�g�̏�������
	//dReal bx = 0.1; dReal by = 0.3; dReal bz = 0.05;
	drawlander();
	

	//�r���{�b�g�̏�ԗʂ̍X�V


	//�r���{�b�g��[�ƒn�ʂƂ̋����v��
	calcDist();

	//�͌v�Z
	//����

	//�v�Z���ʂ����O�Ƃ��ĕۑ�
	copyLogData();
	

	dSpaceCollide(space, 0, &nearCallback);  // �Փˌ��o�֐�

	dWorldStep(world, ONE_STEP);
	dJointGroupEmpty(contactgroup); // �W���C���g�O���[�v����ɂ���

	sim.steps++;
	if (sim.steps > SIM_CNT_MAX) { dsStop(); }

}

void setDrawStuff()           /*** �`��֐��̐ݒ� ***/
{
	fn.version = DS_VERSION;    // �h���[�X�^�b�t�̃o�[�W����
	fn.start = &start;        // �O���� start�֐��̃|�C���^
	fn.step = &simLoop;      // simLoop�֐��̃|�C���^
	fn.command = &command; //�R�}���h�֐��̃|�C���^
	fn.path_to_textures = "C:/ode-0.13/drawstuff/textures"; // �e�N�X�`��
}

//���t�����txt�t�@�C���Ƀf�[�^��������
void saveData() {
	time_t timer;
	struct tm now;
	struct tm *local;
	timer = time(NULL);
	localtime_s(&now, &timer);
	strftime(file_name, 256, FILE_PATH, &now);
	FILE *fp;
	fp = fopen(file_name, "w");
	for (size_t i = 0; i < SIM_CNT_MAX; i++)
	{
		fprintf(fp, "%f ", sim.times[i]);
		fprintf(fp, "%f ", sim.heights[i]);
		for (size_t j = 0; j < LEG_NUM; j++)
		{
			fprintf(fp, "%f ", sim.log_leglen[i][j]);
			fprintf(fp, "%f ", sim.log_legvel[i][j]);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}

//txt�t�@�C�����̃f�[�^��GNUPLOT�ŃO���t��
void saveGraph() {
	FILE *gp;
	if ((gp = _popen(GNUPLOT_PATH, "w")) == NULL) { printf("Can not find %s!", GNUPLOT_PATH);}
	else { printf("GNUPLOT activates.\n"); }
	fprintf(gp, "pl \"%s\" us 1:2 w l\n", file_name);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);
	fprintf(gp, "set term wxt 1\n");
	fprintf(gp, "pl \"%s\" us 1:3 w l, \"%s\" us 1:5 w l\n", file_name,file_name);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);
	fflush(gp); // �o�b�t�@�Ɋi�[����Ă���f�[�^��f���o���i�K�{�j
	_pclose(gp);
}

//���C�����[�v
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
	system("pause");
	return 0;
}