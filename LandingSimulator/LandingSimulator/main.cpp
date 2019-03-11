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
#include "BMP.h"
#include "main.h"
#include "controlrobot.h"

#ifdef dDOUBLE                      // �P���x�Ɣ{���x�̗����ɑΉ�����
#define dsDrawSphere dsDrawSphereD  // ���߂̂��܂��Ȃ�
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
#define FOLDER_PATH "data/%y%m%d_%H%M%S"
#define LOG_FILE "/logs.txt"
#define BMP_FILE "/imgs"
#define FILENAME_GRAPH1 "/img_jnt_pos.png"
#define FILENAME_GRAPH2 "/img_leg_length.png"
#define FILENAME_GRAPH3 "/img_ctrl_force.png"
#define FILENAME_GRAPH4 "/img_ext_force.png"

#define ALL_LOG
#define BMP_FLG 1
#define FORCE_SENSOR
#define SLS_CONTROL

using namespace std;

dWorldID world;  // ���͊w�v�Z�p���[���h
dSpaceID space;  // �Փˌ��o�p�X�y�[�X
dGeomID  ground; // �n��
dJointGroupID contactgroup; // �R���^�N�g�O���[�v
dsFunctions fn;
dJointID sjoint[LEG_NUM];//�X���C�_�[�W���C���g
dJointID fixed[LEG_NUM];//�r���{�b�g��Body�̌Œ�
dJointID forcefixed[LEG_NUM];//�r���{�b�g�Ɨ̓Z���T�̌Œ�
dJointFeedback feedback[LEG_NUM];//�̓Z���T����̃t�B�[�h�o�b�N

dReal F[LEG_NUM][3];//�̓Z���T����o�͂�����
dReal Tau[LEG_NUM];//�����

MyObject body, leg[LEG_NUM], piston[LEG_NUM],forcesensor[LEG_NUM];//�錾
char file_name[256];
char folder_name[256];
char picfolder[256];

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
#ifdef FORCE_SENSOR
	destroyforcesensor();
	makeforcesensor();
#endif // FORCE_SENSOR

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
			sim.legpos[i][j] = dBodyGetPosition(piston[i].body)[j];
		}
		sim.Rot[i] = dBodyGetRotation(piston[i].body);
		dReal temp[3];
		dReal vec[3];
		vec[2] = piston[i].l / 2.0;
		dMultiply0(temp, sim.Rot[i], vec, 3, 3, 1);
		sim.dist[i] = sim.legpos[i][2] - temp[2];
	}
}

//�n�ʂƂ̋������狁�߂鉼�z��
void ctrlLeg() {
	//������
	if (sim.steps==0)
	{
		//�C���s�[�_���X�p�����[�^�ݒ�
		//����́C�����b�O�ŋϓ��ɐݒ�
		for (size_t i = 0; i < LEG_NUM; i++)
		{
			piston[i].imp.m = piston[i].m;
			piston[i].imp.C = 10.00;
			piston[i].imp.K = 52.0;
			piston[i].imp.K0 = 30.0;
		}

	}
	//SLS�ɂ��쓮�͌v�Z
	calcSLS(&sim);

	//�쓮�͓���
	for (size_t i = 0; i < LEG_NUM; i++)
	{
		dJointAddSliderForce(sjoint[i], Tau[i]);
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
			sim.log_Tau[sim.steps][i] = Tau[i];
			sim.log_Fz[sim.steps][i] = F[i][2];
		}
	}
}

//ODE�V�~�����[�V�������[�v
static void simLoop(int pause) {
	//UAV�I�u�W�F�N�g�̏�������
	//dReal bx = 0.1; dReal by = 0.3; dReal bz = 0.05;
	drawlander();
#ifdef FORCE_SENSOR
	drawforcesensor();
#endif // FORCE_SENSOR

	//BMP�t�@�C���̏o��
	if (BMP_FLG == 1 && ((sim.steps >= 0) && (sim.steps < SIM_CNT_MAX))) {

		// BMP�t�@�C���o�͎��ԊԊu�iFPS�ɑ����j
		if (sim.steps%(int)(BMP_STEP/ONE_STEP)==0) {
			WriteBMP(640, 480);	// BMP�t�@�C�����o��
		}
	}


	//�r���{�b�g�̏�ԗʂ̍X�V
	dJointFeedback *fb;
	for (size_t i = 0; i < LEG_NUM; i++)
	{
		fb = dJointGetFeedback(forcefixed[i]);
		for (int j = 0; j < 3; j++)
		{
			F[i][j] = -fb->f2[j];//�̓Z���T�Ōv�����ꂽ�s�X�g���ɂ������
		}
	}

	//�r���{�b�g��[�ƒn�ʂƂ̋����v��
	calcDist();

	//����͌v�Z�C���
	#ifdef SLS_CONTROL
	ctrlLeg();
	#endif // SLS_CONTROL

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
	//���ԁC���x�C�r1�����C�r2�����C�r1���x�C�r2���x
	//�C�r1����́C�r2����́C�r1�O�́C�r2�O��
	time_t timer;
	struct tm now;
	struct tm *local;
	timer = time(NULL);
	localtime_s(&now, &timer);
	strftime(folder_name, 256, FOLDER_PATH, &now);
	_mkdir(folder_name);
	sprintf(file_name, "%s%s", folder_name, LOG_FILE);
	FILE *fp;
	fp = fopen(file_name, "w");
	for (size_t i = 0; i < SIM_CNT_MAX; i++)
	{
		fprintf(fp, "%f ", sim.times[i]);
		fprintf(fp, "%f ", sim.heights[i]);
		for (size_t j = 0; j < LEG_NUM; j++){fprintf(fp, "%f ", sim.log_leglen[i][j]);}
		for (size_t j = 0; j < LEG_NUM; j++) { fprintf(fp, "%f ", sim.log_legvel[i][j]); }
		for (size_t j = 0; j < LEG_NUM; j++) { fprintf(fp, "%f ", sim.log_Tau[i][j]); }
		for (size_t j = 0; j < LEG_NUM; j++) { fprintf(fp, "%f ", sim.log_Fz[i][j]); }
		fprintf(fp, "\n");
	}
	fclose(fp);
}

//txt�t�@�C�����̃f�[�^��GNUPLOT�ŃO���t��
void saveGraph(char *folder) {
	FILE *gp;
	char graph_filename[256];
	if ((gp = _popen(GNUPLOT_PATH, "w")) == NULL) { printf("Can not find %s!", GNUPLOT_PATH);}
	else { printf("GNUPLOT activates.\n"); }
	fprintf(gp, "pl \"%s\" us 1:2 w l\n", file_name);
	sprintf(graph_filename, "%s%s", folder, FILENAME_GRAPH1);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", graph_filename);//UAV�d�S�ʒu�̃O���t
	fprintf(gp, "set term wxt 1\n");//ID:1�̃E�C���h�E�𐶐�
	fprintf(gp, "pl \"%s\" us 1:3 w l, \"%s\" us 1:4 w l\n", file_name,file_name);
	sprintf(graph_filename, "%s%s", folder, FILENAME_GRAPH2);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", graph_filename);//�r���{�b�g�̒����O���t
	fprintf(gp, "set term wxt 2\n");//ID:2�̃E�C���h�E�𐶐�
	fprintf(gp, "pl \"%s\" us 1:7 w l, \"%s\" us 1:8 w l\n", file_name, file_name);
	sprintf(graph_filename, "%s%s", folder, FILENAME_GRAPH3);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", graph_filename);//�r���{�b�g�̐���̓O���t
	fprintf(gp, "set term wxt 3\n");//ID:3�̃E�C���h�E�𐶐�
	fprintf(gp, "pl \"%s\" us 1:9 w l, \"%s\" us 1:10 w l\n", file_name, file_name);
	sprintf(graph_filename, "%s%s", folder, FILENAME_GRAPH4);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", graph_filename);//�r���{�b�g�̊O�̓O���t
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
#ifdef FORCE_SENSOR
	makeforcesensor();
#endif // FORCE_SENSOR

	#ifdef SLS_CONTROL
	initSLS();
	#endif // SLS_CONTROL

	dsSimulationLoop(argc, argv, 640, 480, &fn);
	
	#ifdef ALL_LOG
	saveData();
	saveGraph(folder_name);
	sprintf(picfolder, "%s%s", folder_name, BMP_FILE);
	_mkdir(picfolder);
	SaveBMP(picfolder, 640, 480);
	#endif // ALL_LOG

	

	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	system("pause");
	return 0;
}