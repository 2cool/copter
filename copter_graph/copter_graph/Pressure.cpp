#include "stdafx.h"
#include "Pressure.h"
#include <math.h>

#include "KalmanFilter.h"



	//double dt = 1.0 / 30; // Time step

	static Eigen::MatrixXd A(3, 3); // System dynamics matrix
	static Eigen::MatrixXd C(1, 3); // Output matrix
	static Eigen::MatrixXd Q(3, 3); // Process noise covariance
	static Eigen::MatrixXd R(1, 1); // Measurement noise covariance
	static Eigen::MatrixXd P(3, 3); // Estimate error covariance
	KalmanFilter *kf;

void Pressure::init()
{




	

	// Discrete LTI projectile motion, measuring position only
	A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
	C << 1, 0, 0;

	// Reasonable covariance matrices
	Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
	R << 5;
	P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
	kf=new KalmanFilter(dt, A, C, Q, R, P);

	// Construct the filter

	Eigen::VectorXd x0(3);
	x0 << 0, 0, -9.81;
	kf->init(0, x0);

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
			Eigen::VectorXd y(1);
			y << altitude;
			kf->update(y);
			altitude = kf->state().transpose()[0];
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