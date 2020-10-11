#include "stdafx.h"
#include "hmc.h"
#include "Telemetry.h"
#include "Mpu.h"

Hmc::Hmc()
{
}


Hmc::~Hmc()
{
}


enum{X,Y,Z};
uint8_t buffer[6];
int16_t mx, my, mz,c_base[3];
float dx, dy, dz;
float base[12] = { -0.011184,-0.032271,0.024606,-0.003847,-0.030249,0.017550,0.005991,-0.017475,0.032292,-0.016693,-0.001853,0.044457 };


void Hmc::parser(byte buf[], int n) {

	heading = *(float*)&buf[n];

}

void Hmc::parser_base(byte buf[], int n) {//save calibr,motor-calibr and angle correction
	int16_t sh[6] = { 645,-450,437,-738,743,-458 };


	sh[0] = *(int16_t*)&buf[n]; n += 2;
	sh[1] = *(int16_t*)&buf[n]; n += 2;
	sh[2] = *(int16_t*)&buf[n]; n += 2;
	sh[3] = *(int16_t*)&buf[n]; n += 2;
	sh[4] = *(int16_t*)&buf[n]; n += 2;
	sh[5] = *(int16_t*)&buf[n]; n += 2;

	base[0] = *(float*)&buf[n]; n += 4;
	base[1] = *(float*)&buf[n]; n += 4;
	base[2] = *(float*)&buf[n]; n += 4;
	base[3] = *(float*)&buf[n]; n += 4;
	base[4] = *(float*)&buf[n]; n += 4;
	base[5] = *(float*)&buf[n]; n += 4;
	base[6] = *(float*)&buf[n]; n += 4;
	base[7] = *(float*)&buf[n]; n += 4;
	base[8] = *(float*)&buf[n]; n += 4;
	base[9] = *(float*)&buf[n]; n += 4;
	base[10] = *(float*)&buf[n]; n += 4;
	base[11] = *(float*)&buf[n]; n += 4;
	yaw_correction_angle = RAD2GRAD* *(float*)&buf[n]; n += 4;
	
	if (sh[0] - sh[1] != 0 && sh[2] - sh[3] != 0 && sh[4] - sh[5] != 0) {
		dx = (float)(sh[0] - sh[1]) * 0.5f;
		c_base[X] = (int16_t)(dx + sh[1]);
		dx = 1.0f / dx;
		dy = (float)(sh[2] - sh[3]) * 0.5f;
		c_base[Y] = (int16_t)(dy + sh[3]);
		dy = 1.0f / dy;
		dz = (float)(sh[4] - sh[5]) * 0.5f;
		c_base[Z] = (int16_t)(dz + sh[5]);
		dz = 1.0f / dz;
	}
}

void Hmc::parser_sens(byte buf[], int n, int cont_bits, bool filter, bool rotate) {
	uint8_t buffer[6];
	int i = 0;
	buffer[i] = buf[n+i];i++;
	buffer[i] = buf[n + i]; i++;
	buffer[i] = buf[n + i]; i++;
	buffer[i] = buf[n + i]; i++;
	buffer[i] = buf[n + i]; i++;
	buffer[i] = buf[n + i]; i++;
	

	int16_t mx = (((int16_t)buffer[0]) << 8) | buffer[1];
	int16_t my = (((int16_t)buffer[4]) << 8) | buffer[5];
	int16_t mz = (((int16_t)buffer[2]) << 8) | buffer[3];

	float tfmy = -(float)(mx - c_base[X]) * dx;
	float tfmx = -(float)(my - c_base[Y]) * dy;
	float tfmz = -(float)(mz - c_base[Z]) * dz;

	float FC = 1;// (filter) ? 0.01 : 1;

	if (cont_bits&1 && !rotate) {
		float kx, ky, kz, k;
		//m0;
		k = tel.m_current[0];
		kx = base[0] * k;
		ky = base[1] * k;
		kz = base[2] * k;

		//m1;
		k = tel.m_current[1];
		kx += base[3] * k;
		ky += base[4] * k;
		kz += base[5] * k;

		//m2
		k = tel.m_current[2];
		kx += base[6] * k;
		ky += base[7] * k;
		kz += base[8] * k;

		//m3
		k = tel.m_current[3];
		kx += base[9] * k;
		ky += base[10] * k;
		kz += base[11] * k;

		tfmx -= kx;
		tfmy -= ky;
		tfmz -= kz;
	}

	fmx += (tfmx - fmx) * FC;
	fmy += (tfmy - fmy) * FC;
	fmz += (tfmz - fmz) * FC;


	float cosPitch = cos(Mpu.pitch*GRAD2RAD);
	float sinRoll = sin(Mpu.roll * GRAD2RAD);
	float cosRoll = cos(Mpu.roll * GRAD2RAD);
	float sinPitch = sin(Mpu.pitch * GRAD2RAD);

	float Xh = fmx * cosPitch - fmz * sinPitch;
	float Yh = fmx * sinRoll * sinPitch + fmy * cosRoll - fmz * sinRoll * cosPitch;

	heading = RAD2GRAD*(float)atan2(Yh, Xh);
	heading += yaw_correction_angle;
	if (heading < 0)
		heading += 360;
	if (heading >= 360)
		heading -= 360;



}
Hmc hmc;