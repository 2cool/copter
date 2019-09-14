#pragma once
#include "graph.h"
enum { mPITCH, mROLL, mrPITCH, mrROLL, mYAW, mGYRO_PITCH, mGYRO_ROLL, mGYRO_YAW, mACCX, mACCY, mACCZ,mMAXACC ,mEX,mEY,mALL_E };
#define M_PI  3.14159265358979323846f
#define RAD2GRAD  57.29578
class Mpu
{
public:
	double newR=0.1;
	double newQ = 0.001;



	void loadmax_min(const int n, const double val, bool simetric = false);

	void init();

	void parser(byte buf[], int n, int len, bool filter);
	void toEulerianAngle();



	int _max_minC[mALL_E] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	float _max[mALL_E], _min[mALL_E];


	float dt, time, rdt;

	uint8_t devStatus;      // return status after each device operation
							//(0 = success, !0 = error)
	uint8_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	int16_t a[3];              // [x, y, z]            accel vector
	int16_t g[3];              // [x, y, z]            gyro vector
	int32_t _q[4];
	int32_t t;
	int16_t c[3];


	int r;
	int initialized = 0;
	int dmpReady = 0;
	float lastval[3];
	int16_t sensors;
	//Quaternion q;
	uint8_t rate;
	float gaccX, gaccY;
	//calibration offsets for MPU6050
	///////////////////////////////////   CONFIGURATION   /////////////////////////////
	//Change this 3 variables if you want to fine tune the skecth to your needs.
	const int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
	const int acel_deadzone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
	const int giro_deadzone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

	int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
	enum { ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset };
	// int16_t offset_[6];
	////////////////////////////////////////////////////////////////////////////////////////
	float fx, fy, fz;
	// float const_gyroRoll0, const_gyroPitch0, const_gyroYaw0;
	uint32_t gyroTime;

	float h_yaw;
	uint8_t gLPF;
	//-----------------
	float m7_accX, m7_accY;
	float windFX, windFY;
	float e_speedX, e_speedY;

	float w_accX, w_accY;
	//------------------
	float yaw_off;
	float f_pitch, f_roll;
	float pitch, roll;

	
	float yaw, yaw_offset;

	float DRAG_K, _0007;

	double acc_callibr_timed;
	float hower_thr, min_thr, fall_thr;
	float e_accZ, e_speedZ, w_accZ;
	double timed;
	float e_accX, e_accY;
	float cor_c_pitch, cor_c_roll;
	double oldmpuTimed;


	float cosYaw, sinYaw;
	int8_t max_g_cnt;

	float temp_deb;
	float faccX, faccY, faccZ;

	bool mpu_calibrated, gyro_calibratioan;
	float accZ, accY, accX, tiltPower, cosPitch, cosRoll, sinPitch, sinRoll;
	float accZnF;
	float tiltPower_CF;
	float maccX, maccZ, maccY;


	float  gyroPitch, gyroYaw, gyroRoll;
	float est_alt, est_speedZ;
	float estX, estY, est_speedX, est_speedY;

};
extern Mpu mpu;
