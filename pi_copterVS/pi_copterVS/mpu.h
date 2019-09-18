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


	 double tiltPower_CF;
	 void log_emu();
	 void log();

	 // MPU control/status vars
	// uint8_t devStatus;      // return status after each device operation
							 //(0 = success, !0 = error)
	 //uint8_t fifoCount;     // count of all bytes currently in FIFO
	 //uint8_t fifoBuffer[64]; // FIFO storage buffer

	 int16_t a[3];              // [x, y, z]            accel vector
	 int16_t g[3];              // [x, y, z]            gyro vector
	 int16_t c[3];
	 double mpu_dt;
	 Quaternion_ q;
	 
	 int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
	MPU6050 accelgyro;
	//-----------------

	double pitch, roll;
	double est_alt_, est_speedZ;
	double estX, estY, est_speedX, est_speedY;


	double yaw,yaw_offset;
	void test_vibration( double x,  double y,  double z);
private:
	double altitude_at_zero,XatZero,YatZero;
	void gyro_calibr();
	void test_Est_Alt();
	void test_Est_XY();
	void rotateCW(double &x, double &y);
	void rotateCCW(double &x, double &y);
	double dt;
 public:
	 inline double get_dt() { return dt; }

	 void getXYRelative2Zero(double&x, double&y) { x -= XatZero; y -= YatZero; }
	// void set_cos_sin_dir();
	// double dir_angle_GRAD, cosDirection, sinDirection;
	 double yaw_correction_angle;
	 double get_Est_X() { return estX-XatZero; }
	 double get_Est_Y() { return estY-YatZero; }
	 double get_Est_SpeedX() { return est_speedX; }
	 double get_Est_SpeedY() { return est_speedY; }
	 double get_Est_SpeedZ() { return est_speedZ; }
	 double get_Est_Alt() { return est_alt_-altitude_at_zero; }
	 void set_XYZ_to_Zero();

	 double vibration;
	 int64_t acc_callibr_time;
	 

	 int64_t time,mpu_time, hmc_time, autopilot_time,  ms5611_time,telem_time,com_time;


	 double cor_c_pitch, cor_c_roll;
	 int64_t oldmpuTime;
	 double cosYaw,sinYaw;
	 void initYaw(const double angle);
	
	 void new_calibration(const bool onlyGyro);
	 
	 double get_yaw();
	 double get_pitch();
	 double get_roll();
	 bool mpu_calibrated,gyro_calibratioan;
	double accZ,accY,accX,tiltPower,cosPitch,cosRoll,sinPitch,sinRoll;
	double w_accX, w_accY;
	double  gyroPitch, gyroYaw, gyroRoll;
	string get_set();
	void set(const float  *ar);
	
    int16_t getGX();
	void init();
	bool loop();
	void setDLPFMode_(const uint8_t f);
};

extern MpuClass Mpu;

#endif



