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
//変数群//
//////////

typedef struct {
	//
	int steps;//シミュレーションステップ数(実際の時刻ではない)
	//変数
	double dist[LEG_NUM];//脚ロボットと地面の距離
	double leglen[LEG_NUM];//脚ロボットの相対長さ(1次元)
	double legpos[LEG_NUM][DIM3];//脚ロボットの絶対位置(3次元)
	double legvel[LEG_NUM];//脚ロボットの相対速度(1次元)
	const dReal * Rot[LEG_NUM];//脚ロボットの回転行列

	//ログ格納用配列
	double heights[SIM_CNT_MAX];//UAVの胴体の高度
	double times[SIM_CNT_MAX];//シミュレーション時間
	double log_leglen[SIM_CNT_MAX][LEG_NUM];//脚ロボットの相対長さ
	double log_legvel[SIM_CNT_MAX][LEG_NUM];//脚ロボットの相対速度
	


}SIM;