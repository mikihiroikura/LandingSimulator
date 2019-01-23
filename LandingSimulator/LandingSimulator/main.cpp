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

#ifdef dDOUBLE                      // 単精度と倍精度の両方に対応する
#define dsDrawSphere dsDrawSphereD  // ためのおまじない
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""	// パスに空白があるため[\"]を前後に追加
#define FILE_PATH "data/%y%m%d_%H%M%S_logs.txt"
#define FILENAME_GRAPH1 "img_jnt_pos.png"
#define FILENAME_GRAPH2 "img_leg_length.png"

using namespace std;

dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup; // コンタクトグループ
dsFunctions fn;
dJointID sjoint[LEG_NUM];//スライダージョイント
dJointID fixed[LEG_NUM];//脚ロボットとBodyの固定

typedef struct {       // MyObject構造体
	dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
	double  l, r, m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;

MyObject body, leg[LEG_NUM], piston[LEG_NUM];
char file_name[256];

float xyz[3] = { 3.0,0.0,1.0 };         // 視点の位置
float hpr[3] = { -180, 0, 0 };          // 視線の方向

SIM sim;

// コールバック関数
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10; // 接触点数の最大値
	dContact contact[N];     // 接触点

	// 接触している物体のどちらかが地面ならisGroundに非0をセット
	int isGround = ((ground == o1) || (ground == o2));

	// 衝突情報の生成 nは衝突点数
	int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if (isGround) {
		for (int i = 0; i < n; i++) {
			contact[i].surface.mode = dContactBounce; // 接触面の反発性を設定
			contact[i].surface.bounce = 0.8;          // 反発係数(0.0から1.0)
			contact[i].surface.bounce_vel = 0.0;      // 反発に必要な最低速度

			// 接触ジョイントの生成
			dJointID c = dJointCreateContact(world, contactgroup,
				&contact[i]);
			// 接触している２つの剛体を接触ジョイントにより拘束
			dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
				dGeomGetBody(contact[i].geom.g2));
		}
	}
}

void start()                                  /*** 前処理　***/
{
	dsSetViewpoint(xyz, hpr);                     // カメラの設定
	sim.steps = 0;
}

//シミュレーションリスタート関数
void restart() {
	sim.steps = 0;
	//破壊
	destroylander();
	dJointGroupDestroy(contactgroup);
	//再生成
	contactgroup = dJointGroupCreate(0);
	makelander();
}

//キーボードのコマンド
static void command(int cmd)
{
	switch (cmd) {
	case 'x': xyz[0] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// x方向
	case 'X': xyz[0] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -x方向
	case 'y': xyz[1] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// y方向
	case 'Y': xyz[1] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -y方向
	case 'z': xyz[2] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// z方向
	case 'Z': xyz[2] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -z方向
	//case 'u':	dBodyAddForce(obj.body, 500.0, 0.0, 0.0);	break;
	case 'r':	restart();	break;
	case 'q':	dsStop();	break;
	default:printf("key missed \n"); break;
	}
}

//地面との距離計算
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

//地面との距離から求める仮想力
void ctrlLeg() {
	//初期化
	if (sim.steps==0)
	{

	}
}

//計算結果をログ配列に格納
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

//ODEシミュレーションループ
static void simLoop(int pause) {
	//UAVオブジェクトの書き込み
	//dReal bx = 0.1; dReal by = 0.3; dReal bz = 0.05;
	drawlander();
	

	//脚ロボットの状態量の更新


	//脚ロボット先端と地面との距離計測
	calcDist();

	//力計算
	//何か

	//計算結果をログとして保存
	copyLogData();
	

	dSpaceCollide(space, 0, &nearCallback);  // 衝突検出関数

	dWorldStep(world, ONE_STEP);
	dJointGroupEmpty(contactgroup); // ジョイントグループを空にする

	sim.steps++;
	if (sim.steps > SIM_CNT_MAX) { dsStop(); }

}

void setDrawStuff()           /*** 描画関数の設定 ***/
{
	fn.version = DS_VERSION;    // ドロースタッフのバージョン
	fn.start = &start;        // 前処理 start関数のポインタ
	fn.step = &simLoop;      // simLoop関数のポインタ
	fn.command = &command; //コマンド関数のポインタ
	fn.path_to_textures = "C:/ode-0.13/drawstuff/textures"; // テクスチャ
}

//日付ありのtxtファイルにデータ書き込み
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

//txtファイル内のデータをGNUPLOTでグラフ化
void saveGraph() {
	FILE *gp;
	if ((gp = _popen(GNUPLOT_PATH, "w")) == NULL) { printf("Can not find %s!", GNUPLOT_PATH);}
	else { printf("GNUPLOT activates.\n"); }
	fprintf(gp, "pl \"%s\" us 1:2 w l\n", file_name);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH1);
	fprintf(gp, "set term wxt 1\n");
	fprintf(gp, "pl \"%s\" us 1:3 w l, \"%s\" us 1:5 w l\n", file_name,file_name);
	fprintf(gp, "set terminal png\n set out \"%s\"\n rep\n", FILENAME_GRAPH2);
	fflush(gp); // バッファに格納されているデータを吐き出す（必須）
	_pclose(gp);
}

//メインループ
int main(int argc, char **argv) {
	setDrawStuff();
	dInitODE();
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -0.098);
	dWorldSetERP(world, 0.9);          // ERPの設定
	dWorldSetCFM(world, 1e-4);         // CFMの設定
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