#include "controlrobot.h"

dReal intx[LEG_NUM];
dReal intF[LEG_NUM];

void initSLS() {
	intx[0] = 0.0; intx[1] = 0.0;
	intF[0] = 0.0; intF[1] = 0.0;
}

void calcSLS(SIM *sim) {
	for (size_t i = 0; i < LEG_NUM; i++)
	{
		//積分器の更新
		intx[i] += sim->leglen[i]*ONE_STEP;
		intF[i] += F[i][2]*ONE_STEP;

		//制御力の計算
		dReal tau;
		tau = -piston[i].imp.K / piston[i].imp.C*piston[i].imp.m*sim->legvel[i]
			- (piston[i].imp.K0 + piston[i].imp.K)*sim->leglen[i]
			- piston[i].imp.K / piston[i].imp.C*piston[i].imp.K0*intx[i]
			+ piston[i].imp.K / piston[i].imp.C*intF[i];

		//代入
		Tau[i] = tau;
	}
}