
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
void BalanceClass::set_pitch_roll_pids(const float kp, const float ki, const float imax) {
	pids[PID_PITCH_RATE].kP(kp);
	pids[PID_PITCH_RATE].kI(ki);
	pids[PID_PITCH_RATE].imax(-imax,imax);

	pids[PID_ROLL_RATE].kP(kp);
	pids[PID_ROLL_RATE].kI(ki);
	pids[PID_ROLL_RATE].imax(-imax,imax);
}


void BalanceClass::init()
{
	max_angle = MAX_ANGLE;
	max_throttle = MAX_THROTTLE;
	min_throttle = MIN_THROTTLE;
	f_[0] = f_[1] = f_[2] = f_[3] = 0;
	cout << "BALANCE INIT\n";
	c_pitch = c_roll = 0;
	Stabilization.init();
	true_throttle = throttle = 0;
	pitch_roll_stabKP = 2;
	propeller_lost[0]= propeller_lost[1] = propeller_lost[2] = propeller_lost[3] = false;
	//set_pitch_roll_pids(0.0017,  0.0001, 0.2);  //very old
	old_time = micros_();

	//set_pitch_roll_pids(0.001, 0.001, 0.3);  // 10
	set_pitch_roll_pids(0.0008, 0.001, 0.3);//9


	yaw_stabKP = 2;

	pids[PID_YAW_RATE].kP(0.0017f);  //setup for 9 prop 
	pids[PID_YAW_RATE].kI(0.0017f);
	pids[PID_YAW_RATE].imax(-0.2,0.2);

	delay(1500);

	
//	Mpu.initYaw(Hmc.heading*RAD2GRAD);

	
#ifdef DEBUG_MODE
	printf( "Heading :%i\n", (int)Hmc.get_headingGrad());
	
#endif

}


string BalanceClass::get_set(){
	
	ostringstream convert;
	
	convert << \
		pids[PID_PITCH_RATE].kP() << "," << \
		pids[PID_PITCH_RATE].kI() << "," << \
		pids[PID_PITCH_RATE].imax() << "," << \
		pitch_roll_stabKP << "," << \
		pids[PID_YAW_RATE].kP() << "," << \
		pids[PID_YAW_RATE].kI() << "," << \
		pids[PID_YAW_RATE].imax() << "," << \
		yaw_stabKP << "," << \
		max_angle;
	
	string ret = convert.str();
	return string(ret);
}



void BalanceClass::set(const float *ar, int n){
	int i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		float t;
		if (n == 0) {
			t = pids[PID_PITCH_RATE].kP();
			Settings.set(ar[i++], t);
			pids[PID_PITCH_RATE].kP(t);
			pids[PID_ROLL_RATE].kP(t);
			
			t = pids[PID_PITCH_RATE].kI();
			Settings.set(ar[i++], t);
			pids[PID_PITCH_RATE].kI(t);
			pids[PID_ROLL_RATE].kI(t);
			
			t = pids[PID_PITCH_RATE].imax();
			Settings.set(ar[i++], t);
			pids[PID_PITCH_RATE].imax(-t,t);
			pids[PID_ROLL_RATE].imax(-t,t);
			

			Settings.set(ar[i++], pitch_roll_stabKP);

			t = pids[PID_YAW_RATE].kP();
			Settings.set(ar[i++], t);
			pids[PID_YAW_RATE].kP(t);
			
			t = pids[PID_YAW_RATE].kI();
			Settings.set(ar[i++], t);
			pids[PID_YAW_RATE].kI(t);
			
			t = pids[PID_YAW_RATE].imax();
			Settings.set(ar[i++], t);
			pids[PID_YAW_RATE].imax(-t,t);
			
			t = yaw_stabKP;
			Settings.set(ar[i++], t);
			yaw_stabKP = t;
			
			t = max_angle;
			Settings.set(ar[i++], t);
			max_angle = constrain(t, 15, 35);
			Stabilization.setMaxAng();
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

		//при лагах в св€зи c_pitch,c_roll обнул€ютс€.


		Log.block_end();
		Log.end();
	}
}
void BalanceClass::PID_reset() {
	pids[PID_PITCH_RATE].reset_I();
	pids[PID_ROLL_RATE].reset_I();
	pids[PID_YAW_RATE].reset_I();
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


void BalanceClass::speed_up_control(float n[]) {
#define START_THROTTHLE 0.15
#define SPEEDUP_CNT 200
	static uint32_t start_cnt[4];
	static float max_thr[4];
	for (int i = 0; i < 4; i++) {
		if (n[i] < START_THROTTHLE) {
			start_cnt[i] = 0;
			max_thr[i] = START_THROTTHLE;
		}
		else {
			if (start_cnt[i] < SPEEDUP_CNT && n[i] > max_thr[i]) {
				n[i] = max_thr[i];
				max_thr[i] = START_THROTTHLE + (float)start_cnt[i] * (0.15 / SPEEDUP_CNT);
				start_cnt[i]++;
			}
		}
	}
	//Debug.dump(n[0], n[1], n[2], n[3]);
	//n[0] = n[1] = n[2] = n[3] = 0;
	//if (ret && ret!=0b1111)
	//	cout << ret << endl;

}









#define MAX_D_ANGLE_SPEED 70
#define MAX_D_YAW_SPEED 70
//#define MAX_POWER_K_IF_MAX_ANGLE_30 1.12













static bool speed_up = true;


bool BalanceClass::loop()
{

	if (!Mpu.loop()) {
		MS5611.loop();
		GPS.loop();
		return false;
	}
	else {
		if (Autopilot.motors_is_on()) { 
/*
			static float secure_throttle = max_throttle;
			if (mega_i2c.motors_overload) {
				secure_throttle -= 0.02;
				if (secure_throttle < HOVER_THROTHLE)
					secure_throttle = HOVER_THROTHLE;
			}
			else {
				secure_throttle += 0.01;
				if (secure_throttle > max_throttle)
					secure_throttle = max_throttle;
			}
	*/		
			const float pK = powerK();
			const float c_min_throttle = min_throttle * pK;
			const float c_max_throttle = (max_throttle * pK > OVER_THROTTLE) ? OVER_THROTTLE : max_throttle * pK;

			if (Autopilot.z_stabState()) {
				true_throttle = pK * Stabilization.Z();
			}
			else {
				true_throttle = Autopilot.get_throttle();

			}

			true_throttle = constrain(true_throttle, c_min_throttle, c_max_throttle);
			throttle = true_throttle;
			const float thr = throttle / Mpu.tiltPower;


			//зробити приоритет оставатис€ на м≥ст≥ перед оставатис€ на висот≥
			//пр≥ор≥тет швидкост≥ меньший за висоту. -це зробленно
			if (thr > OVER_THROTTLE) {
				t_max_angle = RAD2GRAD * acos(throttle / OVER_THROTTLE);
				t_max_angle = constrain(t_max_angle, MIN_ANGLE, max_angle);
				throttle = max_throttle;
			}
			else {
				throttle = thr;
				t_max_angle = max_angle;
			}



			//Debug.load(0, true_throttle, throttle);
			//Debug.dump();
			if (Autopilot.xy_stabState()) {
				Stabilization.XY(c_pitch, c_roll);
			}
			else {
				c_pitch = Autopilot.get_Pitch();
				c_roll = Autopilot.get_Roll();
			}

			c_pitch = constrain(c_pitch, -t_max_angle, t_max_angle);
			c_roll = constrain(c_roll, -t_max_angle, t_max_angle);
			const float maxAngle07 = t_max_angle *0.7f;
			if (fabs(c_pitch) > maxAngle07 || fabs(c_roll) > maxAngle07) {
				float k = (float)(RAD2GRAD*acos(cos(c_pitch*GRAD2RAD)*cos(c_roll*GRAD2RAD)));
				if (k == 0)
					k = t_max_angle;
				if (k > t_max_angle) {
					k = t_max_angle / k;
					c_pitch *= k;
					c_roll *= k;
				}
			}


			static float pitch_stab_output =0;
			static float roll_stab_output = 0;
			static float yaw_stab_output = 0;

#define BAL_F 1

			pitch_stab_output += (f_constrain(pitch_roll_stabKP*(wrap_180(Mpu.get_pitch() - c_pitch)), -MAX_D_ANGLE_SPEED, MAX_D_ANGLE_SPEED)-pitch_stab_output)*BAL_F;
			roll_stab_output += (f_constrain(pitch_roll_stabKP*(wrap_180(Mpu.get_roll() - c_roll)), -MAX_D_ANGLE_SPEED, MAX_D_ANGLE_SPEED)-roll_stab_output)*BAL_F;
			yaw_stab_output += (f_constrain(yaw_stabKP*wrap_180(-Autopilot.get_yaw() - Mpu.get_yaw()), -MAX_D_YAW_SPEED, MAX_D_YAW_SPEED)-yaw_stab_output)*BAL_F;

			const float max_delta = 0.3;
			const float yaw_max_delta = 0.2;
			static float correction = 1;
			//correction += (0.5 / fmin(throttle,0.5) - correction)*0.2;
			const int64_t _ct = micros_();
			double dt = (_ct - old_time) * 1e-6;
			dt = constrain(dt, 0.001, 0.03);
			old_time = _ct;
			float pitch_output = pK*pids[PID_PITCH_RATE].get_pid(correction*(pitch_stab_output + Mpu.gyroPitch), dt);
			pitch_output = constrain(pitch_output, -max_delta, max_delta);
			float roll_output = pK*pids[PID_ROLL_RATE].get_pid(correction*(roll_stab_output + Mpu.gyroRoll), dt);
			roll_output = constrain(roll_output, -max_delta, max_delta);
			float yaw_output = pK*pids[PID_YAW_RATE].get_pid(correction*(yaw_stab_output - Mpu.gyroYaw), dt);
			yaw_output = constrain(yaw_output, -yaw_max_delta, yaw_max_delta);

#ifdef YAW_OFF
			//yaw_output = 0;
			//pitch_output=0;
#endif

			f_[3] = f_constrain((throttle + roll_output + pitch_output - yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[1] = f_constrain((throttle + roll_output - pitch_output + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[2] = f_constrain((throttle - roll_output + pitch_output + yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);
			f_[0] = f_constrain((throttle - roll_output - pitch_output - yaw_output), STOP_THROTTLE_, FULL_THROTTLE_);


			
			

			if (Hmc.do_compass_motors_calibr) {
				f_[0] = f_[1] = f_[2] = f_[3] = 0;
				f_[Hmc.motor_index] = HOVER_THROTHLE;
				throttle = HOVER_THROTHLE;
			}
			else {
				const int32_t speedup_time = 5e3;
				const int32_t _ct32 = _ct / 1e3;
				if ((_ct32 - Autopilot.time_at__start) < speedup_time || (Autopilot.time_at__start - Autopilot.old_time_at__start) > 8e3) {
					true_throttle = MIN_THROTTLE;
					f_[0] = f_[1] = f_[2] = f_[3] = throttle = true_throttle;
					Autopilot.hall_test();//put only there
				}


				//f_[0] = f_[1] = f_[2] = f_[3] = throttle;//!!!!!!!!!!!!!!!!!


				
			}
			if (speed_up && throttle > HOVER_THROTHLE)
				speed_up = false;

		}else
			speed_up = true;
		
		
//#define MOTORS_OFF
#ifdef DEBUG
#ifdef MOTORS_OFF
		f_[0] = f_[1] = f_[2] = f_[3] = 0;
#endif
#endif
	/*	f_[0] = 0;
		f_[1] = 0;
		f_[2] = 0;
		f_[3] = 0;*/
		
		speed_up_control(f_);
		if (speed_up)
			PID_reset();
		mega_i2c.throttle(f_);  //670 micros
		log();
	}

	return true;
}

void BalanceClass::set_off_th_() { 
	f_[0] = f_[1] = f_[2] = f_[3] = 0; 
	throttle = true_throttle = 0;
	mega_i2c.throttle(f_);
}








BalanceClass Balance;

