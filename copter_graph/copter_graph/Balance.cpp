#include "stdafx.h"
#include "Balance.h"

int Balance::decode(char buffer[], int &i, bool rotate) {
	i += 9;
	f[bcPITCH]=*(float*)(&buffer[i]);
	i += 4;
	f[bcROLL] = *(float*)(&buffer[i]);
	i+= 4;
	f[bTHROTHLE] = *(float*)(&buffer[i]);
	i += 4;
	int n=*(int*)(&buffer[i]);
	i += 4;
	i += 2;
	f[bF0] = *(float*)(&buffer[i]);
	i += 4;
	f[bF1] = *(float*)(&buffer[i]);
	i += 4;
	f[bF2] = *(float*)(&buffer[i]);
	i += 4;
	f[bF3] = *(float*)(&buffer[i]);
	i += 4;

	return 0;
}
void Balance::init()
{
	for (int i = 0; i < bALL; i++) {
		_max[i] = -1000000;
		_min[i] =  1000000;
	}


}


void Balance::parser(byte buf[], int i) {

	int bank_counter = *(uint32_t*)&buf[i];// load_int32(buf, i);
	i += 4;
	float *fb = (float*)&buf[i];
	f0 = fb[0];
	f1 = fb[1];
	f2 = fb[2];
	f3 = fb[3];
	thr = (f0 + f1 + f2 + f3)*0.25;

	i += 16;
	ap_roll = 1.0 / 16 * (double)(*(int16_t*)&buf[i]);
	i += 2;
	ap_pitch = 1.0 / 16 * (double)(*(int16_t*)&buf[i]); ;
														  
	i += 2;
	ap_yaw = 1.0 / 16 * (double)(*(int16_t*)&buf[i]); ;



}





Balance bal;


