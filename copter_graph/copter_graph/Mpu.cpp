#include "stdafx.h"
#include "Mpu.h"
#include "Pressure.h"
#include "GPS_Loger.h"
#include "Pressure.h"
#include "KalmanFilter.h"

class Pendulum {
private:
	
	float mass, stiff;
	float k;
	float q;
public:
	float possition;
	float  speed;
	Pendulum(float m, float s, float _k,float q_) {
		k =0.2;
		possition = 0;
		mass = m;
		speed = 0;
		stiff = 90;
		q = q_;
	}
	void loop(float pos) {

		const float dt = 0.01;
		float force = possition * stiff - (pos - possition)*k;
		force += (speed > 0) ? q : -q;

		float a = force / mass;
		speed -= a * dt;
		
		possition += speed * dt;


	}


};

int mn = 3; // Number of states
int m = 1; // Number of measurements

double dt = 1.0 / 30; // Time step

Eigen::Matrix3d A(mn, mn); // System dynamics matrix
Eigen::RowVector3d H(m, mn); // Output matrix
Eigen::Matrix3d Q(mn, mn); // Process noise covariance
Eigen::MatrixXd R(m, m); // Measurement noise covariance
Eigen::Matrix3d P(mn, mn); // Estimate error covariance


enum {X,Y,Z};
KalmanFilter* kf[3];


#define ROLL_COMPENSATION_IN_YAW_ROTTATION 0.02
#define PITCH_COMPENSATION_IN_YAW_ROTTATION 0.025

float ac_accX = 0, ac_accY = 0, ac_accZ = 1.15504527;
float agpitch = 0, agroll = 0, agyaw = 0;

uint64_t maxG_firs_time = 0;


bool compas_flip = false;


#define _2PI 6.283185307179586476925286766559


bool pitch_flag;
bool set_yaw_flag = true;

inline void sin_cos(const float a, float &s, float &c) {
	s = (float)sin(a);
	c = (float)cos(a);
	/*
	const double ss = s*s;
	c = (float)sqrt(1 - min(1.0f, ss));
	//30.7.2017 corected
	if (abs(a) > 90)
	c = -c;
	*/
}




//////////////////////////////////////////////////////////////////////////////////////
int load_uint8(byte buf[], int i) {
	int vall = buf[i];
	vall &= 255;
	//if (vall<0)
	//	vall=0-vall;
	return vall;
}

uint64_t loaduint64t(byte buf[], int i) {

	uint64_t  *ip = (uint64_t*)&buf[i];

	return *ip;

}




int32_t load_int32(byte buf[], int i) {

	int32_t *ip = (int32_t*)&buf[i];

	return *ip;
}
int16_t load_int16(byte buf[], int i) {

	int16_t *ip = (int16_t*)&buf[i];
	return *ip;
}


const float n003 = 0.030517578f;
const float n006 = 0.061035156f;
//4g
const float n122 = 1.220740379e-4;
double qw, qx, qy, qz,g_yaw;

void Mpu::toEulerianAngle()
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (qw * qx + qy * qz);
	double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);

	roll = RAD2GRAD * atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (qw * qy - qz * qx);
	if (abs(sinp) >= 1)
		pitch = RAD2GRAD * copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = RAD2GRAD * asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (qw * qz + qx * qy);
	double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
	g_yaw = RAD2GRAD * atan2(siny, cosy);
}

void Mpu::loadmax_min(const int mn, const double val, bool simetric) {



	if (_max_minC[mn] == 0) {
		_max[mn] = _min[mn] = val;
		_max_minC[mn]++;
	}
	else {
		_max[mn] = max(mpu._max[mn], val);
		_min[mn] = min(mpu._min[mn], val);
		if (simetric) {
			_max[mn] = max(mpu._max[mn], -val);
			_min[mn] = min(mpu._min[mn], -val);
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pendulum p0(1,0, 0, 0.3);
Pendulum p1(1.01, 0, 0,0);




#include <iostream>
using namespace std;




void Mpu::parser(byte buf[], int j, int len, bool filter) {


	len += j;

	static double old_time = 0;
	
	uint64_t itime = loaduint64t(buf, j);
	j += 8;
	time = (double)itime *0.001;
	//if (time > 302)
	//	time--;
	dt = time - old_time;
	old_time = time;

	
	int g[3];
	int a[3];
	int q[4];

	if (dt > 0.01)
		dt = 0.01;

	kf[Z]->A <<
		1, dt, 0,
		0, 1, dt,
		0, 0, 1;
	memcpy(g, &(buf[j]), 6);
	j += 6;
	memcpy(a, &(buf[j]), 6);
	j += 6;
	//memcpy(q, &(buf[j]), 16);
	//j += 16;

	pitch= *(float*)&buf[j]; j += 4;
	roll= *(float*)&buf[j]; j += 4;
	yaw = *(float*)&buf[j]; j += 4;;
	gyroPitch = *(float*)&buf[j]; j += 4;
	gyroRoll = *(float*)&buf[j]; j += 4;
	gyroYaw = *(float*)&buf[j]; j += 4;

#define ACC_CF 0.007

	const float f = 1;// (filter ? ACC_CF : 1);
	if (j <= len) { 
		const float tacc =  *(float*)& buf[j];
		j += 4;
		accX += (tacc - accX)*f; 
	}
	if (j <= len) {
		const float tacc = *(float*)& buf[j];
		j += 4;
		accY += (tacc - accY)*f; 
	}
	if (j <= len) {
		accZnF = *(float*)& buf[j];	
		j += 4;

		accZ += (accZnF - accZ)*f;
		//accZ = *(float*)&buf[j];
	}
	

	if (j <= len) { 
		est_alt = *(float*)&buf[j]; j += 4; }
	if (j <= len) { 
		est_speedZ = *(float*)&buf[j]; j += 4; }

	if (j <= len) {
		estX = *(float*)& buf[j]; j += 4;
	}
	if (j <= len) {
		est_speedX = *(float*)& buf[j]; j += 4;
	}
	if (j <= len) {
		estY = *(float*)& buf[j]; j += 4;
	}
	if (j <= len) {
		est_speedY = *(float*)& buf[j]; j += 4;
	}


	static float old_accZ = 0,old_accX=0,old_accY=0;
	static double old_bar_alt = 0,oldSX=0,oldSY=0;
	if (filter) {

		
		//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ


		kf[Z]->B[2] = accZ - old_accZ;
		old_accZ = accZ;
		if (press.altitude != old_bar_alt) { 
			old_bar_alt = press.altitude;
			Eigen::VectorXd y(m);
			y << old_bar_alt;
			kf[Z]->update(y);
		}
		else {
			kf[Z]->update();
		}
		est_speedZ = kf[Z]->state()(1);
		est_alt = kf[Z]->state()(0);
		accZ = kf[Z]->state()(2);
		//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

		kf[0]->B[2] =  accX - old_accX;
		old_accX = accX;
		if (gps_log.gx != oldSX) {
			oldSX = gps_log.gx;
			Eigen::VectorXd y(m);
			y << oldSX;
			kf[0]->update(y);
		}
		else {
			kf[0]->update();
		}
		est_speedX = kf[0]->state()(1);
		estX = kf[0]->state()(0);
		accX = kf[0]->state()(2);


		//YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY

		kf[1]->B[2] = accY - old_accY;
		old_accY = accY;
		if (gps_log.gy != oldSY) {
			oldSY = gps_log.gy;
			Eigen::VectorXd y(m);
			y << oldSY;
			kf[1]->update(y);
		}
		else {
			kf[1]->update();
		}
		est_speedY = kf[1]->state()(1);
		estY = kf[1]->state()(0);
		accY = kf[1]->state()(2);
	}




	_max[mPITCH] = 60;
	_min[mPITCH] = -60;
	_max[mROLL] = 60;
	_min[mROLL] = -60;
	_max[mYAW] = 180;
	_min[mYAW] = -180;

	_max[mACCX] = 5;
	_min[mACCX] = -5;
	_max[mACCY] = 5;
	_min[mACCY] = -5;
	_max[mACCZ] = 5;
	_min[mACCZ] = -5;


/*
	loadmax_min(mPITCH, pitch,true);
	loadmax_min(mROLL, roll, true);
	loadmax_min(mYAW, yaw, true);

	loadmax_min(mACCX, accX, true);
	loadmax_min(mACCY, accY, true);
	loadmax_min(mACCZ, accZ, true);
*/
	
	loadmax_min(SZ, est_alt, true);

	loadmax_min(mEX,estX);
	loadmax_min(mEY,estY);


}


void Mpu::init() {
	// Discrete LTI projectile motion, measuring position only
	A <<1, 0.005, 0,0, 1, 0.005,0, 0, 1;
	H << 1, 0, 0;
	// Reasonable covariance matrices
	Q <<newQ, newQ, .0,newQ, newQ, .0,.0, .0, .0;
	R << newR;
	P <<.1, .1, .1,.1, 10000, 10,.1, 10, 100;
	Vector3d B(mn);
	B << 0, 0, 0;
	// Construct the filter
	kf[X] = new KalmanFilter(A, B, H, Q, R, P);
	kf[Y] = new KalmanFilter(A, B, H, Q, R, P);
	kf[Z] = new KalmanFilter( A,B ,H, Q, R, P);
	// Construct the filter
	Eigen::VectorXd x0(3);
	x0 << 0, 0, 0;
	kf[X]->init(x0);
	kf[Y]->init(x0);
	kf[Z]->init(x0);
}





Mpu mpu;