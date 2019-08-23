#include "define.h"
#include "KalmanFilter.h"
// mpu.h

#ifndef _MPU_h
#define _MPU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "I2Cdev.h"
#include "MPU6050.h"
#include "helper_3dmath.h"
//#include "MotionSensor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


#include "define.h"

#ifdef FLY_EMULATOR
#include "mpu_umulator.h"
#endif

//===================================================

//#define SESOR_UPSIDE_DOWN
class MpuClass 
{
	friend class HmcClass;
 protected:


	 float tiltPower_CF;
	 void log_emu();
	 void log();

	 // MPU control/status vars
	// uint8_t devStatus;      // return status after each device operation
							 //(0 = success, !0 = error)
	 //uint8_t fifoCount;     // count of all bytes currently in FIFO
	 //uint8_t fifoBuffer[64]; // FIFO storage buffer

	 int16_t a[3];              // [x, y, z]            accel vector
	 int16_t g[3];              // [x, y, z]            gyro vector
	 int32_t t;
	 int16_t c[3];
	 float mpu_dt;

	 int r;
	 int initialized = 0;
	 int dmpReady = 0;
	 float lastval[3];
	 int16_t sensors;
	 Quaternion_ q;
	 //calibration offsets for MPU6050
	 ///////////////////////////////////   CONFIGURATION   /////////////////////////////
	 //Change this 3 variables if you want to fine tune the skecth to your needs.
	 const int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
	 const int acel_deadzone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
	 const int giro_deadzone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
	 
	 int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
	 enum{ ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset };
	// int16_t offset_[6];
	 ////////////////////////////////////////////////////////////////////////////////////////

	MPU6050 accelgyro;

	uint8_t gLPF;
	//-----------------

	float yaw_off;
	float pitch, roll;
	float est_alt_, est_speedZ;
	float estX, estY, est_speedX, est_speedY;


	float yaw,yaw_offset;
	void test_vibration( float x,  float y,  float z);
	float _0007;
private:
	float altitude_at_zero,XatZero,YatZero;
	void gyro_calibr();
	void test_Est_Alt();
	void test_Est_XY();
	void rotateCW(float &x, float &y);
	void rotateCCW(float &x, float &y);
	double dt;
 public:
	 inline double get_dt() { return dt; }
	 float dist2home();
	 void getXYRelative2Zero(float&x, float&y) { x -= XatZero; y -= YatZero; }
	// void set_cos_sin_dir();
	// double dir_angle_GRAD, cosDirection, sinDirection;
	 float yaw_correction_angle;
	 float get_Est_X() { return estX-XatZero; }
	 float get_Est_Y() { return estY-YatZero; }
	 float get_Est_SpeedX() { return est_speedX; }
	 float get_Est_SpeedY() { return est_speedY; }
	 float get_Est_SpeedZ() { return est_speedZ; }
	 float get_Est_Alt() { return est_alt_-altitude_at_zero; }
	 void set_XYZ_to_Zero();

	 float vibration;
	 int64_t acc_callibr_time;
	 

	 int64_t time,mpu_time, gps_time, hmc_time, autopilot_time,  ms5611_time,telem_time,com_time;


	 float cor_c_pitch, cor_c_roll;
	 int64_t oldmpuTime;


	 float cosYaw,sinYaw;
	 int8_t max_g_cnt;

	 float temp_deb;
	 void initYaw(const float angle);
	
	 void new_calibration(const bool onlyGyro);
	 
	 float get_yaw();
	 float get_pitch();
	 float get_roll();
	 bool mpu_calibrated,gyro_calibratioan;
	float accZ,accY,accX,tiltPower,cosPitch,cosRoll,sinPitch,sinRoll;
	float w_accX, w_accY;
	float  gyroPitch, gyroYaw, gyroRoll;
	string get_set();
	void set(const float  *ar);
	
    int16_t getGX();
	void init();
	bool loop();
	void setDLPFMode_(const uint8_t f);
};

extern MpuClass Mpu;

#endif



