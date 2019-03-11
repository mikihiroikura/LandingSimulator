#pragma once

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>

using namespace std;

#define ONE_STEP 0.01
#define BMP_STEP 0.1
#define SIM_CNT_MAX 6000
#define LEG_NUM 2
#define DIM3 3


//////////
//�ϐ��Q//
//////////

typedef struct {
	double m, C, K, K0;//SLS model�̃p�����[�^�Q
	//m:�d��,C:�S��,K:�e��,K0:�e��(SLS����΂�)
}Impedance;

typedef struct {       // MyObject�\����
	dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	double  l, r, m;       // ����[m], ���a[m]�C����[kg]
	Impedance imp;		 //SLS���f���̃C���s�[�_���X�p�����[�^(piston�̂�)
} MyObject;


typedef struct {
	//
	int steps;//�V�~�����[�V�����X�e�b�v��(���ۂ̎����ł͂Ȃ�)
	//�ϐ�
	double dist[LEG_NUM];//�r���{�b�g�ƒn�ʂ̋���
	double leglen[LEG_NUM];//�r���{�b�g�̑��Β���(1����)
	double legpos[LEG_NUM][DIM3];//�r���{�b�g(�s�X�g��)�̐�Έʒu(3����)
	double legvel[LEG_NUM];//�r���{�b�g�̑��Α��x(1����)
	const dReal * Rot[LEG_NUM];//�r���{�b�g�̉�]�s��
	double Fout[LEG_NUM][DIM3];//�r���{�b�g�ɂ�����O��

	//���O�i�[�p�z��
	double heights[SIM_CNT_MAX];//UAV�̓��̂̍��x
	double times[SIM_CNT_MAX];//�V�~�����[�V��������
	double log_leglen[SIM_CNT_MAX][LEG_NUM];//�r���{�b�g�̑��Β���
	double log_legvel[SIM_CNT_MAX][LEG_NUM];//�r���{�b�g�̑��Α��x
	dReal log_Tau[SIM_CNT_MAX][LEG_NUM];//�r���{�b�g�ւ̐����
	dReal log_Fz[SIM_CNT_MAX][LEG_NUM];//�r���{�b�g��[�ɂ������
	

}SIM;