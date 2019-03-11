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
		//Ï•ªŠí‚ÌXV
		intx[i] += sim->leglen[i]*ONE_STEP;
		intF[i] += F[i][2]*ONE_STEP;

		//§Œä—Í‚ÌŒvŽZ
		dReal tau;
		tau = -piston[i].imp.K / piston[i].imp.C*piston[i].imp.m*sim->legvel[i]
			- (piston[i].imp.K0 + piston[i].imp.K)*sim->leglen[i]
			- piston[i].imp.K / piston[i].imp.C*piston[i].imp.K0*intx[i]
			+ piston[i].imp.K / piston[i].imp.C*intF[i];

		//‘ã“ü
		Tau[i] = tau;
	}
}