#pragma once

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include <vector>

using namespace std;

#define ONE_STEP 0.01
#define SIM_CNT_MAX 6000
#define LEG_NUM 2
#define DIM3 3


//////////
//�ϐ��Q//
//////////

typedef struct {
	//
	int steps;//�V�~�����[�V�����X�e�b�v��(���ۂ̎����ł͂Ȃ�)
	//�ϐ�
	double dist[LEG_NUM];//�r���{�b�g�ƒn�ʂ̋���
	double leglen[LEG_NUM];//�r���{�b�g�̑��Β���(1����)
	double legpos[LEG_NUM][DIM3];//�r���{�b�g�̐�Έʒu(3����)
	double legvel[LEG_NUM];//�r���{�b�g�̑��Α��x(1����)
	const dReal * Rot[LEG_NUM];//�r���{�b�g�̉�]�s��

	//���O�i�[�p�z��
	double heights[SIM_CNT_MAX];//UAV�̓��̂̍��x
	double times[SIM_CNT_MAX];//�V�~�����[�V��������
	double log_leglen[SIM_CNT_MAX][LEG_NUM];//�r���{�b�g�̑��Β���
	double log_legvel[SIM_CNT_MAX][LEG_NUM];//�r���{�b�g�̑��Α��x
	


}SIM;