
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






class Pendulum {
private:
	float possition, speed;
	float mass, stiff;
	float k;
public:
	
	Pendulum(float m, float s, float _k) {
		k = 0.1;
		possition = 1;
		mass = 1;
		speed = 10;
		stiff = 1;
	}
	void loop(float pos) {
		const float dt = 0.01;
		float force = possition * stiff + (pos-possition)*k;
		float a = force / mass;
		speed -= a * dt;
		

	}


};






void correct(float & f){
	if (f < 0)
		f = 0;
	else if (f>1)
		f = 1;
}




/*



если скорость радиан в секунду то 0.11

63 грама

2.24-3.01 за 108.088 милесекунды

*/




/*

0         1

+

2    +    3

*/






float MAX_ANGLE__ = 35;

float old[2];
float force[2];
float balanceForce[2];

float dTime[2];
float nTime[2];
#define MIN_SPEED 0.2f
float const predTime = 0.009f;



//float dt = 0.009f;

long oldtttttttttttt = 0;
int cntttttttttt = 0;

//bool tempp1 = false;
//uint32_t taim0;
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


//P_R_rateKP","P_R_rateKI","P_R_rateIMAX","P_R_stabKP","YAW_rate_KP","YAW_rateE_KI","YAW_rate_IMAX","YAW_stab_KP","MAX_ANGLE"},




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


	//set_pitch_roll_pids(0.001, 0.001, 0.3);  // 10
	set_pitch_roll_pids(0.0012, 0.001, 0.3);//9


	yaw_stabKP = 2;

	pids[PID_YAW_RATE].kP(0.0017f);  //setup for 9 prop 
	pids[PID_YAW_RATE].kI(0.0017f);
	pids[PID_YAW_RATE].imax(-0.2,0.2);

	delay(1500);

	Mpu.init();
	Hmc.init();
	Hmc.loop();
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
void BalanceClass::reset() {
	pids[PID_PITCH_RATE].reset_I();
	pids[PID_ROLL_RATE].reset_I();
	pids[PID_YAW_RATE].reset_I();
	c_pitch = c_roll = 0;
	Stabilization.resset_xy_integrator();
	Stabilization.resset_z();
}

bool BalanceClass::set_min_max_throttle(const float max, const float min) {
	if (min < MIN_THROTTLE || max > MAX_THROTTLE)
		return true;
	max_throttle = max;
	min_throttle = min;
	Stabilization.setMinMaxI_Thr();




	return false;
}


/////////////////////////////////////////////////////////////////////////////////////////

#define MAX_D_ANGLE_SPEED 70
#define MAX_D_YAW_SPEED 70
//#define MAX_POWER_K_IF_MAX_ANGLE_30 1.12



bool BalanceClass::loop()
{
	
	//double c_timed = Mpu.timed;
	if (!Mpu.loop()) {
		MS5611.loop();
		GPS.loop();
		return false;
	}
	else {
		if (Autopilot.motors_is_on()) { 

			const float pK =  powerK();
			const float c_min_throttle = min_throttle*pK;
			const float c_max_throttle = (max_throttle*pK > OVER_THROTTLE) ? OVER_THROTTLE : max_throttle * pK;

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
		
			float pitch_output = pK*pids[PID_PITCH_RATE].get_pid(correction*(pitch_stab_output + Mpu.gyroPitch), Mpu.dt);
			pitch_output = constrain(pitch_output, -max_delta, max_delta);
			float roll_output = pK*pids[PID_ROLL_RATE].get_pid(correction*(roll_stab_output + Mpu.gyroRoll), Mpu.dt);
			roll_output = constrain(roll_output, -max_delta, max_delta);
			float yaw_output = pK*pids[PID_YAW_RATE].get_pid(correction*(yaw_stab_output - Mpu.gyroYaw), Mpu.dt);
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
				f_[Hmc.motor_index] = 0.5;
			}
			else {
#ifndef FLY_EMULATOR
				if (Mpu.timed - Autopilot.time_at_startd < 5 || Autopilot.time_at_startd - Autopilot.old_time_at_startd > 8) {
					f_[0] = f_[1] = f_[2] = f_[3] = throttle = true_throttle = 0.3;//
					//if (Mpu.vibration > 3)
						//Autopilot.off_throttle(true, "VBR");

				}
#endif
				if (throttle < MIN_THROTTLE) {
					reset();
				}

			}

		}
		else {
			reset();
		}
		
//#define MOTORS_OFF
#ifdef MOTORS_OFF
		f_[0] = f_[1] = f_[2] = f_[3] = 0;
#endif
		//f_[0] = f_[1] = f_[2] = f_[3] = 0.51;
		//f_[0] = f_[1] = 0.502;
		//отключить двигатели при слабом токе
		//if (propeller_lost[0] || propeller_lost[3]) 	f_[0]=f_[3] = 0;
		//if (propeller_lost[1] || propeller_lost[2]) 	f_[1] = f_[2] = 0;
	//	if (f_[0]>=0.4 || f_[1]>0.4 || f_[2]>0.4 || f_[3]>0.4)	f_[0] = f_[1] = f_[2] = f_[3] = throttle;
		//f_[3] = f_[0];

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

