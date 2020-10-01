
#include "Balance.h"
#include "MPU6050.h"
#include "define.h"
#include "Stabilization.h"
#include "Telemetry.h"
#include "debug.h"
#include "GPS.h"
#include "Log.h"
#include "mpu.h"
#include "Settings.h"


static const float f_constrain(const float v, const float min, const float max){
	return constrain(v, min, max);
}

float BalanceClass::set_max_throthle_without_limit(const float angl) { 
	max_angle = angl; 
	Stabilization.setMaxAngels(); 
}


void BalanceClass::max_ang(const float ang, float& angX, float& angY) {
	float k = sqrt(angX * angX + angY * angY);
	if (k > ang) {
		k = ang / k;
		angX *= k;
		angY *= k;
	}
}


void BalanceClass::init()
{
	alt_over_place = true;
	max_angle = MAX_ANGLE;
	max_throttle = MAX_THROTTLE;
	min_throttle = MIN_THROTTLE;
	f_[0] = f_[1] = f_[2] = f_[3] = 0;
	cout << "BALANCE INIT\n";
	c_pitch = c_roll = 0;
	Stabilization.init();
	throttle = 0;
	propeller_lost[0]= propeller_lost[1] = propeller_lost[2] = propeller_lost[3] = false;
	old_time = micros_();
	pid.kP(0.0016);
	pid.set_kI(0.00065);

	pid.set_kI_max(MAX_DELTA);
	pid.hi_2_error_max_diff(MAX_DELTA);
	pitch_roll_kD = 0.00065;
	yaw_pid.kP(0.004f);  
	yaw_pid.kI(0.004f);
	yaw_pid.imax(-MAX_YAW_DELTA, MAX_YAW_DELTA);
	yaw_kD =  0.004;
	delay(1500);
//	Mpu.initYaw(Hmc.heading*RAD2GRAD);

	
#ifdef DEBUG_MODE
	printf( "Heading :%i\n", (int)Hmc.get_headingGrad());
	
#endif

}


string BalanceClass::get_set(){
	
	ostringstream convert;
	
	convert << \
		pid.kP() << "," << \
		pid.get_kI() << "," << \
		pitch_roll_kD << "," << \
		pid.get_kI_max() << "," << \
		yaw_pid.kP() << "," << \
		yaw_pid.kI() << "," << \
		yaw_kD << "," << \
		yaw_pid.imax() << "," << \
		max_angle;
	
	string ret = convert.str();
	return string(ret);
}



void BalanceClass::set(const float *ar, int n){
	int i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		float t;
		if (n == 0) {
			t = pid.kP();
			Settings.set(ar[i++], t);
			pid.kP(t);
			t = pid.get_kI();
			Settings.set(ar[i++], t);
			pid.set_kI(t);
			Settings.set(ar[i++], pitch_roll_kD);
			t = pid.get_kI_max();
			Settings.set(ar[i++], t);
			pid.set_kI_max(t);
			t = yaw_pid.kP();
			Settings.set(ar[i++], t);
			yaw_pid.kP(t);
			t = yaw_pid.kI();
			Settings.set(ar[i++], t);
			yaw_pid.kI(t);
			Settings.set(ar[i++], yaw_kD);
			t = yaw_pid.imax();
			Settings.set(ar[i++], t);
			yaw_pid.imax(-t,t);
			t = max_angle;
			Settings.set(ar[i++], t);
			max_angle = constrain(t, 15, 45);
			Stabilization.setMaxAngels();
		}
	}
	cout << "balance set:\n";
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;
}

float BalanceClass::powerK(){
	return (Telemetry.powerK*MS5611.powerK);
	
}

void BalanceClass::log() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::BAL);
		Log.write_bank_cnt();
		Log.loadMem((uint8_t*)f_, 16, false);
		Log.loadInt16t((int)c_roll * 16);
		Log.loadInt16t((int)c_pitch * 16);
		Log.loadInt16t((int)Autopilot.get_yaw() * 16);
		Log.block_end();
		Log.end();
	}
}
void BalanceClass::PID_reset() {
	pid.reset_integrators();

	yaw_pid.reset_I();
	c_pitch = c_roll = 0;
	Stabilization.resset_xy_integrator();
	Stabilization.resset_z();
}

bool BalanceClass::set_min_max_throttle(const float max, const float min) {
	min_throttle = (min < MIN_THROTTLE) ? MIN_THROTTLE : min;
	max_throttle = (max > MAX_THROTTLE) ? MAX_THROTTLE : max;
	Stabilization.setMinMaxI_Thr();




	return false;
}


/////////////////////////////////////////////////////////////////////////////////////////


bool BalanceClass::speed_up_control(float n[]) {
#define START_THROTTHLE 0.01
#define SPEEDUP_CNT 1000
	bool speed_up = false;
	static uint32_t start_cnt[4];
	static float max_thr[4];
	for (int i = 0; i < 4; i++) {
		if (n[i] < START_THROTTHLE) {
			start_cnt[i] = 0;
			max_thr[i] = START_THROTTHLE;
			speed_up = true;
		}
		else {
			if (start_cnt[i] < SPEEDUP_CNT && n[i] > max_thr[i]) {
				n[i] = max_thr[i];
				max_thr[i] = START_THROTTHLE + (float)start_cnt[i] * ((MIN_THROTTLE - START_THROTTHLE) / SPEEDUP_CNT);
				if (max_thr[i] > MIN_THROTTLE)
					max_thr[i] = MIN_THROTTLE;
				start_cnt[i]++;
				speed_up = true;
			}
		}
	}
	return speed_up;
}

#define MAX_D_ANGLE_SPEED 180
#define MAX_D_YAW_SPEED 180

bool speed_up = true;
bool BalanceClass::loop()
{
	if (Autopilot.motors_is_on()) { 
	
		const float pK = powerK();
		float _thr;
		if (Autopilot.z_stabState()) {
			_thr = throttle = pK * Stabilization.Z();
			throttle /= Mpu.tiltPower;
			const float c_min_throttle = min_throttle * pK;
			if (throttle < c_min_throttle)
				throttle = c_min_throttle;

			if (throttle > OVER_THROTTLE) {
				if (_thr >= OVER_THROTTLE) {
					if (alt_over_place && Mpu.get_Est_SpeedZ() < 0 && Mpu.get_Est_Alt() + 5 < Autopilot.fly_at_altitude()) { //drone is falling
						throttle = OVER_THROTTLE;
						t_max_angle = MIN_ANGLE;
					}
					else {
						throttle = OVER_THROTTLE;
						t_max_angle = max_angle;
					}
				}
				else {
					throttle = OVER_THROTTLE;
					t_max_angle = RAD2GRAD * acos(_thr / OVER_THROTTLE);
				}
			}else
				t_max_angle = max_angle;
		}
		else {
			_thr = throttle = Autopilot.get_throttle();
			throttle /= Mpu.tiltPower;
			const float c_min_throttle = 0.35 * pK;
			throttle = f_constrain(throttle, c_min_throttle, OVER_THROTTLE);
			t_max_angle = max_angle;
		}

		if (Autopilot.xy_stabState()) {
			Stabilization.XY(c_pitch, c_roll);
		}
		else {
			c_pitch = Autopilot.get_Pitch();
			c_roll = Autopilot.get_Roll();
		}

		max_ang(t_max_angle, c_pitch, c_roll);

		const float pitch_error = wrap_180(Mpu.get_pitch() - c_pitch);
		const float roll_error = wrap_180(Mpu.get_roll() - c_roll);
		const float yaw_error =  (Autopilot.if_strong_wind() && Autopilot.go2homeState()) ? 0 :  wrap_180(-Autopilot.get_yaw() - Mpu.get_yaw());

		const int64_t _ct = micros_();
		double dt = (_ct - old_time) * 1e-6;
		dt = constrain(dt, 0.001, 0.03);
		old_time = _ct;
#define _PITCH 0
#define _ROLL 1
		float *output = pid.get_pid(pitch_error, roll_error, Mpu.get_dt());
		output[_PITCH] = pK * (output[_PITCH]+ Mpu.gyroPitch * pitch_roll_kD);
		output[_ROLL]  = pK * (output[_ROLL] + Mpu.gyroRoll  * pitch_roll_kD);

		//float pitch_output = pK * (pids[PID_PITCH_RATE].get_pid(pitch_error, dt) + Mpu.gyroPitch*pitch_roll_kD);
		output[_PITCH] = constrain(output[_PITCH], -MAX_DELTA, MAX_DELTA);

		//float roll_output = pK * (pids[PID_ROLL_RATE].get_pid(roll_error , dt) + Mpu.gyroRoll*pitch_roll_kD);
		output[_ROLL] =  constrain(output[_ROLL], -MAX_DELTA, MAX_DELTA);

		float yaw_output =  pK* (yaw_pid.get_pid(yaw_error, dt) - Mpu.gyroYaw * yaw_kD);
		yaw_output =  constrain(yaw_output, -MAX_YAW_DELTA, MAX_YAW_DELTA);

#ifdef YAW_OFF
		//yaw_output = 0;
		//pitch_output=0;
#endif
		float m_yaw_output = -yaw_output;  //антираскачивание при низкой мощности на плече
		if ((throttle + yaw_output) < min_throttle)
			yaw_output = min_throttle - throttle;
		if ((throttle + m_yaw_output) < min_throttle)
			m_yaw_output = min_throttle - throttle;

		f_[3] = f_constrain((throttle + output[_ROLL] + output[_PITCH] + m_yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
		f_[1] = f_constrain((throttle + output[_ROLL] - output[_PITCH] + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
		f_[2] = f_constrain((throttle - output[_ROLL] + output[_PITCH] + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
		f_[0] = f_constrain((throttle - output[_ROLL] - output[_PITCH] + m_yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);







		//float* i = new float[4];i = pid.get_integrator();Debug.dump(i[0], i[2], i[1], i[3]);

	//	Debug.dump(c_roll,  c_pitch, 0, 0);



		if (Hmc.do_compass_motors_calibr) {
			f_[0] = f_[1] = f_[2] = f_[3] = 0;
			f_[Hmc.motor_index] = throttle = Hmc.throttle;
		}
		else {
#ifndef FLY_EMULATOR
			const int32_t speedup_time = 5e3; 
			const int32_t _ct32 = _ct / 1e3;
			if ((_ct32 - Autopilot.time_at__start) < speedup_time || (Autopilot.time_at__start - Autopilot.old_time_at__start) > 8e3) {
				f_[0] = f_[1] = f_[2] = f_[3] = throttle = MIN_THROTTLE;
				Autopilot.hall_test();//put only there
			}
#endif
			/*
			if (throttle > 0.05)
				throttle = 0.05;
			f_[0] = f_[1] = f_[2] = f_[3] = throttle;//!!!!!!!!!!!!!!!!!
			*/
			//f_[1] = f_[2] = f_[3] = 0;
		}
	}
	else
		speed_up = true;
		
		
//#define MOTORS_OFF
#ifdef DEBUG
#ifdef MOTORS_OFF
	f_[0] = f_[1] = f_[2] = f_[3] = 0;
#endif
#endif

//	f_[0] = f_[1] = f_[2] = f_[3] = 0;
	if (speed_up && (speed_up_control(f_) || throttle <= MIN_THROTTLE_2_DONT_RESET_STAB_PIDS))
		PID_reset();
	else
		speed_up = false;

	mega_i2c.throttle(f_);  //670 micros
	log();
	return true;
}

void BalanceClass::set_off_th_() { 
	f_[0] = f_[1] = f_[2] = f_[3] = throttle = 0; 
	mega_i2c.throttle(f_);
}








BalanceClass Balance;

