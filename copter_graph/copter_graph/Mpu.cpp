#include "stdafx.h"
#include "Mpu.h"
#include "Pressure.h"
#include "GPS_Loger.h"
#include "Pressure.h"
#include "KalmanFilter.h"

#include "hmc.h"
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

Eigen::Matrix3f A; // System dynamics matrix
Eigen::RowVector3f H; // Output matrix
Eigen::Matrix3f Q; // Process noise covariance
Eigen::MatrixXf R(m, m); // Measurement noise covariance
Eigen::Matrix3f P; // Estimate error covariance


enum {X,Y,Z,X1,Y1,X2,Y2};
KalmanFilter* kf[7];


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
void MPU_CLASS::toEulerianAngle(const Quaternion_& q, float& roll, float& pitch, float& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);

	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	yaw = atan2(siny, cosy);
}
void MPU_CLASS::toEulerianAngle()
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

void MPU_CLASS::loadmax_min(const int mn, const double val, bool simetric) {



	if (_max_minC[mn] == 0) {
		_max[mn] = _min[mn] = val;
		_max_minC[mn]++;
		
	}
	else {
		_max[mn] = max(_max[mn], val);
		_min[mn] = min(_min[mn], val);
		if (simetric) {
			_max[mn] = max(_max[mn], -val);
			_min[mn] = min(_min[mn], -val);
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Pendulum p0(1,0, 0, 0.3);
Pendulum p1(1.01, 0, 0,0);




#include <iostream>
using namespace std;

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352
#define wrap_PI(x) (x < -PI ? x+TWO_PI : (x > PI ? x - TWO_PI: x))






void MPU_CLASS::Madgwic() {
	const float giroifk = 0.06103515625f;//2000
	const float accifk = 0.00048828125f;//16

	gyroPitch = giroifk * (float)g[1] - agpitch;
	gyroRoll = giroifk * (float)g[0] - agroll;
	gyroYaw = giroifk * (float)g[2] - agyaw;
	float ax = accifk * (float)a[0];
	float ay = accifk * (float)a[1];
	float az = accifk * (float)a[2];

	AHRS.MadgwickAHRSupdate(q, GRAD2RAD * gyroRoll, GRAD2RAD * gyroPitch, GRAD2RAD * gyroYaw, ax, ay, az, hmc.fmx, hmc.fmy, hmc.fmz, dt);
	toEulerianAngle(q, roll, pitch, yaw);

	gyroPitch = -gyroPitch;
	gyroYaw = -gyroYaw;
	pitch = -pitch;
	yaw = -yaw;
	yaw += hmc.yaw_correction_angle*GRAD2RAD;
	yaw = wrap_PI(yaw);

	sin_cos(yaw, sinYaw, cosYaw);
	sin_cos(pitch, sinPitch, cosPitch);
	sin_cos(roll, sinRoll, cosRoll);

	tiltPower += (constrain(cosPitch * cosRoll, 0.5f, 1) - tiltPower) * tiltPower_CF;

	accZ = az * cosPitch + sinPitch * ax;
	accZ = 9.8f * (accZ * cosRoll + sinRoll * ay - 1);

	accX = 9.8f * (ax * cosPitch - az * sinPitch);
	accY = 9.8f * (-ay * cosRoll + az * sinRoll);



	pitch *= RAD2GRAD;
	roll *= RAD2GRAD;
	yaw *= RAD2GRAD;
}


//-------------------------------------------------------------
void test1(const float w_accX, const float w_accY) {
	static float old_accX = 0, old_accY = 0;
	static float oldSX = 0, oldSY = 0;
	

	kf[X1]->B[2] = w_accX - old_accX;
	old_accX = w_accX;
	if (gps_log.gx != oldSX) {
		oldSX = gps_log.gx;
		Eigen::VectorXf x(m);
		x << oldSX;
		kf[X1]->update(x);
	}
	else {
		kf[X1]->update();
	}
	//YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
	kf[Y1]->B[2] = w_accY - old_accY;
	old_accY = w_accY;
	if (gps_log.gy != oldSY) {
		oldSY = gps_log.gy;
		Eigen::VectorXf y(m);
		y << oldSY;
		kf[Y1]->update(y);
	}
	else {
		kf[Y1]->update();
	}
}

//-------------------------------------------------------------
void test2(const float w_accX, const float w_accY) {
	static float old_accX = 0, old_accY = 0;
	static float oldSX = 0, oldSY = 0;


	kf[X2]->B[2] = w_accX - old_accX;
	old_accX = w_accX;
	if (gps_log.gx != oldSX) {
		oldSX = gps_log.gx;
		Eigen::VectorXf x(m);
		x << oldSX;
		kf[X2]->update(x);
	}
	else {
		kf[X2]->update();
	}
	//YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY
	kf[Y2]->B[2] = w_accY - old_accY;
	old_accY = w_accY;
	if (gps_log.gy != oldSY) {
		oldSY = gps_log.gy;
		Eigen::VectorXf y(m);
		y << oldSY;
		kf[Y2]->update(y);
	}
	else {
		kf[Y2]->update();
	}
}
float error_comp = 0;
void MPU_CLASS::parser(byte buf[], int j, int len, int cont_bits, bool filter, bool rotate) {


	len += j;

	static uint64_t old_itime = 0;

	uint64_t itime = loaduint64t(buf, j);
	j += 8;
	time = (double)itime * 0.001;
	dt = 0.001 * (itime - old_itime);
	old_itime = itime;




	if (dt > 0.03)
		dt = 0.03;

	kf[Z]->A <<
		1, dt, 0,
		0, 1, dt,
		0, 0, 1;



	kf[X]->A(3) = kf[X]->A(7) = dt;
	kf[Y]->A(3) = kf[Y]->A(7) = dt;
	kf[X1]->A(3) = kf[X1]->A(7) = dt;
	kf[Y1]->A(3) = kf[Y1]->A(7) = dt;	
	kf[X2]->A(3) = kf[X2]->A(7) = dt;
	kf[Y2]->A(3) = kf[Y2]->A(7) = dt;



	memcpy(g, &(buf[j]), 6);
	j += 6;
	memcpy(a, &(buf[j]), 6);
	j += 6;
	//memcpy(q, &(buf[j]), 16);
	//j += 16;

	pitch = *(float*)&buf[j]; j += 4;
	roll = *(float*)&buf[j]; j += 4;
	yaw = *(float*)&buf[j]; j += 4;;
	if (yaw < 0)
		yaw += 360;

	cosYaw = cos((yaw+ error_comp) * GRAD2RAD);
	sinYaw = sin((yaw+ error_comp) * GRAD2RAD);


	gyroPitch = *(float*)&buf[j]; j += 4;
	gyroRoll = *(float*)&buf[j]; j += 4;
	gyroYaw = *(float*)&buf[j]; j += 4;

#define ACC_CF 1
	static double ACCX = 0, ACCY = 0, ACCZ = 0;
	const float f =  (filter ? ACC_CF : 1);
	if (j <= len) {
		const float tacc = *(float*)&buf[j];
		j += 4;
		ACCX += (tacc - ACCX) * f;
	}
	if (j <= len) {
		const float tacc = *(float*)&buf[j];
		j += 4;
		ACCY += (tacc - ACCY) * f;
	}
	if (j <= len) {
		accZnF = *(float*)&buf[j];
		j += 4;

		ACCZ += (accZnF - ACCZ) * f;
		//accZ = *(float*)&buf[j];
	}
	accX = ACCX;
	accY = ACCY;
	accZ = ACCZ;

	if (j <= len) {
		est_alt = *(float*)&buf[j]; j += 4;
	}
	if (j <= len) {
		est_speedZ = *(float*)&buf[j]; j += 4;
	}

	if (j <= len) {
		estX = *(float*)&buf[j]; j += 4;
	}
	if (j <= len) {
		est_speedX += (*(float*)&buf[j] - est_speedX) * f; j += 4;
	}
	if (j <= len) {
		estY = *(float*)&buf[j]; j += 4;
	}
	if (j <= len) {
		est_speedY += (*(float*)&buf[j] - est_speedY) * f; j += 4;
	}



	


	static float old_accZ = 0, old_accX = 0, old_accY = 0;
	static float old_bar_alt = 0, oldSX = 0, oldSY = 0;
	static bool start_f = true;





	w_accX = (-cosYaw * accX + sinYaw * accY); //relative to world
	w_accY = (-cosYaw * accY - sinYaw * accX);

	if (start_f == false && gps_log.gx != 0 && gps_log.gy != 0)
		start_f = true;

/*
	if (filter) {
		
		Madgwic();
		if (yaw < 0)
			yaw += 360;
	}
	*/
	 
	//if (time < 200 )
	//	error_comp = -180;
	
	if (start_f && filter && time>200 && time < 496) {


		//ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ

		static float f_accZ = 0;
		f_accZ += (accZ - f_accZ) * 1;
		accZ = f_accZ;
		kf[Z]->B[2] = accZ - old_accZ;
		old_accZ = accZ;
		if (press.altitude != old_bar_alt) {
			old_bar_alt = press.altitude;
			Eigen::VectorXf z(m);
			z << old_bar_alt;
			kf[Z]->update(z);
		}
		else {
			kf[Z]->update();
		}
		est_speedZ = kf[Z]->state()(1);
		est_alt = kf[Z]->state()(0);
		accZ = kf[Z]->state()(2);
		//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

		kf[0]->B[2] = w_accX - old_accX;
		old_accX = w_accX;
		if (gps_log.gx != oldSX) {
			oldSX = gps_log.gx;
			Eigen::VectorXf x(m);
			x << oldSX;
			kf[0]->update(x);
		}
		else {
			kf[0]->update();
		}
		est_speedX = kf[0]->state()(1);
		estX = kf[0]->state()(0);
		w_accX = kf[0]->state()(2);


		//YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY

		kf[1]->B[2] = w_accY - old_accY;
		old_accY = w_accY;
		if (gps_log.gy != oldSY) {
			oldSY = gps_log.gy;
			Eigen::VectorXf y(m);
			y << oldSY;
			kf[1]->update(y);
		}
		else {
			kf[1]->update();
		}
		est_speedY = kf[1]->state()(1);
		estY = kf[1]->state()(0);
		w_accY = kf[1]->state()(2);
		//////////////////////////////////////


		

		cosYaw = cos((yaw - 45 + error_comp) * GRAD2RAD);
		sinYaw = sin((yaw - 45 + error_comp) * GRAD2RAD);
		float _w_accX = (-cosYaw * accX + sinYaw * accY); //relative to world
		float _w_accY = (-cosYaw * accY - sinYaw * accX);
		test1(_w_accX, _w_accY);

		cosYaw = cos((yaw + 45 + error_comp) * GRAD2RAD);
		sinYaw = sin((yaw + 45 + error_comp) * GRAD2RAD);
		_w_accX = (-cosYaw * accX + sinYaw * accY); //relative to world
		_w_accY = (-cosYaw * accY - sinYaw * accX);
		test2(_w_accX, _w_accY);

		static float  er1 = 0, er2 = 0;

		//er0 += (abs(GPS.loc.speedX - est_speedX) + abs(GPS.loc.speedY - est_speedY) - er0) * 0.01;
		
		er1 += (abs(gps_log.gspeedX - kf[X + 3]->state()(1)) + abs(gps_log.gspeedY - kf[Y + 3]->state()(1)) - er1) * 1;
		er2 += (abs(gps_log.gspeedX - kf[X + 5]->state()(1)) + abs(gps_log.gspeedY - kf[Y + 5]->state()(1)) - er2) * 1;

			if (abs(gps_log.gspeedX) > 0.3 || abs(gps_log.gspeedY) > 0.3) {
				if (er1 > er2)
					error_comp += (er1 - er2) * 0.01f ;
				else if (er1 < er2)
					error_comp -= (er2 - er1) * 0.01f;
	
		
		
			
				if (error_comp > 360)
					error_comp -= 360;
				else if (error_comp < -360)
					error_comp += 360;
			}
		estX = yaw+error_comp;

			//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

		










		///////////////////////////////////



	}

	if (rotate == false) {
		accX = (-cosYaw * w_accX - sinYaw * w_accY); //relative to world
		accY = (-cosYaw * w_accY + sinYaw * w_accX);
	}
	else {
		accX = w_accX;
		accY = w_accY;

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
	if (cont_bits & 1) {
		loadmax_min(SZ, est_alt, true);

		loadmax_min(mEX, estX);
		loadmax_min(mEY, estY);
	}
	else {
		start_pos[SZ] = est_alt;
		start_pos[mEX] = estX;
		start_pos[mEY] = estY;

	}

	/*
	float npitch = (float)(mpu.cosYaw * pitch + mpu.sinYaw * roll);
	float nroll = (float)(mpu.cosYaw * roll - mpu.sinYaw * pitch);
	pitch = npitch;
	roll = nroll;
	*/

	est_LF_X_speed += (est_speedX - est_LF_X_speed) * 0.1;
	est_LF_Y_speed += (est_speedY - est_LF_Y_speed) * 0.1;

	est_LF_X_ACC += (accX - est_LF_X_ACC) * 0.1;
	est_LF_Y_ACC += (accY - est_LF_Y_ACC) * 0.1;


	est_LF_VER_speed += (est_speedZ - est_LF_VER_speed) * 0.1;
	est_LF_VER_ACC += (accZ - est_LF_VER_ACC) * 0.1f;

	/*if (filter) {
		accX = accY = get_est_LF_hor_acc();
		est_speedY = est_speedX = get_est_LF_hor_speed();
		accZ = est_LF_VER_ACC;
		est_speedZ = est_LF_VER_speed;
	}*/



}


void MPU_CLASS::init() {
	// Discrete LTI projectile motion, measuring position only
	A <<1, 0.005, 0,0, 1, 0.005,0, 0, 1;
	H << 1, 0, 0;
	// Reasonable covariance matrices
	Q <<newQ, newQ, .0,newQ, newQ, .0,.0, .0, .0;
	R << newR;
	P <<.1, .1, .1,.1, 10000, 10,.1, 10, 100;
	Vector3f B;
	B << 0, 0, 0;
	// Construct the filter
	kf[X] = new KalmanFilter(A, B, H, Q, R, P);
	kf[Y] = new KalmanFilter(A, B, H, Q, R, P);
	kf[Z] = new KalmanFilter( A,B ,H, Q, R, P);

	kf[X1] = new KalmanFilter(A, B, H, Q, R, P);
	kf[Y1] = new KalmanFilter(A, B, H, Q, R, P);
	kf[X2] = new KalmanFilter(A, B, H, Q, R, P);
	kf[Y2] = new KalmanFilter(A, B, H, Q, R, P);




	// Construct the filter
	Eigen::Vector3f x0;
	x0 << 0, 0, 0;
	kf[X]->init(x0);
	kf[Y]->init(x0);
	kf[Z]->init(x0);

	kf[X1]->init(x0);
	kf[Y1]->init(x0);
	kf[X2]->init(x0);
	kf[Y2]->init(x0);

	q.w = 1; q.x = q.y = q.z = 0;
}





MPU_CLASS Mpu;