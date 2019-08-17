#include "stdafx.h"
#include "Pressure.h"
#include <math.h>






void Pressure::init()
{






}
int Pressure::view(int &indexes, char buffer[], int &i) {
	indexes++;

	//float pressur = *(float*)(&buffer[i + 1]);
	i += 5;

	return 0;
}


int Pressure::decode(char buffer[], int &i)
{

	float pressure = *(float*)(&buffer[i + 1]);
	
#define PRESSURE_AT_0 101325
	double t = (44330.0f * (1.0f - pow((double)pressure / PRESSURE_AT_0, 0.1902949f)));

	if (f && cf_alt*(dt * 100)<=1)
		t_alt += (t - t_alt)*cf_alt*(dt * 100);
	else
		t_alt = t;





	max_alt = max(max_alt, t_alt);
	min_alt = min(min_alt, t_alt);
	if (dt < 0.02)
		dt = 0.02;
	if (f && cf_sp*(dt * 100)<=1)
		t_sp += ((t_alt - alt) / dt - t_sp)*cf_sp*(dt*100);
	else
		t_sp = (t_alt - alt) / dt;
	if (t_sp > 10)
		t_sp = 10;
	if (t_sp < -5)
		t_sp = -5;
	min_sp = min(min_sp, t_sp);
	max_sp = max(max_sp, t_sp);
	alt = t_alt;
	if (f && cf_acc*(dt * 100)<=1)
		acc += ((t_sp - speed) / dt - acc)*cf_acc*(dt * 100);
	else
		acc = (t_sp - speed) / dt;
	if (acc > 20)
		acc = 20;
	if (acc < -10)
		acc = -10;
	min_a = min(min_a, acc);
	max_a = max(max_a, acc);
	speed = t_sp;
	dt = 0;
	i += 5;
	return 0;
}




void Pressure::parser(byte buf[], int n, bool filter) {
	float alt=0;
	temp = buf[n];
	n++;
	float pf = *(float*)&buf[n];
	static double talt = 0;
	static double old_alt = 0, told_alt1 = 0, told_alt2 = 0;
	pressure = pf;
	dt = 0.02;
	static double t_alt = 0;
	if (pressure > 80000 && pressure < 120000) {
		altitude = (44330.0f * (1.0f - pow(pressure / PRESSURE_AT_0, 0.1902949f)));
		if (filter) {
			
		}
		if (old_alt == 0)
			old_alt=t_alt=told_alt2=told_alt1 = altitude;

		t_alt += (altitude - t_alt)*0.007;

		speed = ( altitude - old_alt) / dt;
		acc = (t_alt - 2 * told_alt1 + told_alt2) / (dt*dt);
		told_alt2 = told_alt1;
		told_alt1 = t_alt;
		old_alt = altitude;

		
		

		max_alt = max(max_alt, altitude);
		min_alt = min(min_alt, altitude);
	}
	








}

Pressure press;