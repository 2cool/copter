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
	 void log();

	 // MPU control/status vars
	// uint8_t devStatus;      // return status after each device operation
							 //(0 = success, !0 = error)
	 //uint8_t fifoCount;     // count of all bytes currently in FIFO
	 //uint8_t fifoBuffer[64]; // FIFO storage buffer

	 int16_t a[3];              // [x, y, z]            accel vector
	 int16_t g[3];              // [x, y, z]            gyro vector

	 float mpu_dt;
	 Quaternion_ q;
	 
	 int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
	MPU6050 accelgyro;
	//-----------------

	float pitch, roll;
	float est_alt, est_speedZ,est_accZ,est_accY,est_accX;
	float estX, estY, est_speedX, est_speedY;
	float est_LF_X_speed, est_LF_Y_speed, est_LF_X_ACC, est_LF_Y_ACC, est_LF_VER_speed, est_LF_VER_ACC;


	float yaw,yaw_offset;
	void test_vibration(float x, float y, float z);
private:
	int base_z, base_y, base_x;
	float alt_at_zero,x_at_zero,y_at_zero;
	void gyro_acc_calibr(const float&ax, const float &ay, const float &az);
	void test_Est_Alt();
	void test_Est_XY();
	void test_Est_XY1(const float cosYaw, const float sinYaw);
	void test_Est_XY2(const float cosYaw, const float sinYaw);

	void rotateCW(float&x, float&y);
	void rotateCCW(float&x, float&y);
	void acc_gps_yaw_correciton();

 public:
	 float get_est_LF_hor_abs_speed() { return sqrt(est_LF_X_speed * est_LF_X_speed + est_LF_Y_speed * est_LF_Y_speed); }
	 float get_est_LF_hor_abs_acc() { return sqrt(est_LF_X_ACC * est_LF_X_ACC + est_LF_Y_ACC * est_LF_Y_ACC); }
	 float get_est_LF_ver_abs_speed() { return abs(est_LF_VER_speed); }
	 float get_est_LF_ver_abs_acc() { return abs(est_LF_VER_ACC); }

	 inline float get_dt() { return mpu_dt; }
	 inline float est_speed() { return sqrt(estX * estX + estY * estY); }
	 void getXYRelative2Zero(double&x, double&y) { x -= x_at_zero; y -= y_at_zero; }
	// void set_cos_sin_dir();
	// double dir_angle_GRAD, cosDirection, sinDirection;
	 float yaw_correction_angle;
	 double get_Est_X_() { return (double)(estX-x_at_zero)+base_x; }
	 double get_Est_Y_() { return (double)(estY-y_at_zero)+base_y; }
	 float get_Est_accX_() { return est_accX; }
	 float get_Est_accY_() { return est_accY; }
	 float get_Est_accZ() { return est_accZ; }

	 float get_Est_SpeedX_() { return est_speedX; }
	 float get_Est_SpeedY_() { return est_speedY; }
	 float get_Est_SpeedZ() { return est_speedZ; }
	 float get_Est_SpeedXY_() { return sqrt(est_speedX * est_speedX + est_speedY * est_speedY); }
	 double get_Est_Alt() { return (double)(est_alt-alt_at_zero)+base_z; }
	 void set_XYZ_to_Zero();

	 float vibration;
	 int64_t acc_callibr_time;
	 

	 int64_t time,mpu_time, hmc_time, autopilot_time,  ms5611_time,telem_time,com_time;


	 float cor_c_pitch, cor_c_roll;
	 float cosYaw,sinYaw;
	 void initYaw(const double angle);
	
	 void new_calibration(const bool onlyGyro);
	 
	 float get_yaw();
	 float get_pitch();
	 float get_roll();
	 bool mpu_calibrated,gyro_calibratioan;
	 float accZ,accY,accX,tiltPower,cosPitch,cosRoll,sinPitch,sinRoll;
	 float accZ_c, accY_c, accX_c;
	 float w_accX, w_accY;
	 float  gyroPitch, gyroYaw, gyroRoll;
	string get_set();
	void set(const float  *ar);
	
    int16_t getGX();
	bool init();
	bool loop();
	void setDLPFMode_(const uint8_t f);
};

extern MpuClass Mpu;

#endif



