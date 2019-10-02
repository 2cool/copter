// Balance.h

#ifndef _BALANCE_h
#define _BALANCE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Hmc.h"

#include "mi2c.h"
#include "mpu.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Autopilot.h"
#include "AP_PID.h"

/*
    ^
0   |    1
 \  |  /
  \   /
   000
   000
  /   \
 /     \
2       3

*/

class BalanceClass
{
 protected:
	 void PID_reset();
	 void log();
	 int64_t old_time;
	 void set_pitch_roll_pids(const float kp, const float ki, const float imax);
	float throttle,true_throttle;
	//float pitch_roll_stabKP, pitch_roll_rateKP, yaw_rateKP,yaw_stabKP,pitch_roll_rateKI,pitch_roll_rateIMAX,yaw_rateKI,yaw_rateIMAX;
	//float getThrottle();
	float yaw_stabKP, pitch_roll_stabKP;

	float f_[4];

	float t_max_angle;
	float max_throttle,min_throttle, max_angle;
	bool speed_up_control(float n[]);
 public:
	 float get_max_throttle() { return max_throttle; }
	 float get_min_throttle() { return min_throttle; }
	 float get_max_angle() { return max_angle; }

	 bool propeller_lost[4];
	 
	 float powerK();
	 float c_pitch, c_roll;
	 float get_throttle(){ return throttle; }
	 float get_true_throttle() { return true_throttle; }
	 float gf(const uint8_t n){ return f_[n]; }
	 float gf0(){ return f_[0]; }
	 float gf1(){ return f_[1]; }
	 float gf2(){ return f_[2]; }
	 float gf3(){ return f_[3]; }
	// float get_throttle(){ return (throttle-1000)*0.001; }

	 bool set_min_max_throttle(const float max, const float min);
	 
	 




string get_set();
void set(const float  *ar,int n=0);

	void init();
	bool loop();



	void set_off_th_();
	float pitch2rollK;
#define BALANCE_PIDS 3
	AP_PID pids[BALANCE_PIDS];
private:

#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_YAW_RATE 2



	float accXaccY_K;
	

};

extern BalanceClass Balance;

#endif

