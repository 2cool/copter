// 
// 
// 
#include "WProgram.h"
#include "define.h"
#include "WProgram.h"
#include "Stabilization.h"
#include "mpu.h"
#include "mpu.h"
#include "MS5611.h"
#include "Autopilot.h"

#include "GPS.h"
#include "debug.h"
#include "Balance.h"
#include "Prog.h"
#include "Log.h"
#include "Settings.h"

float Z_FILTER = 0.3f;
float XY_FILTER = 0.76f;



float windRC;
float wind_i_X, wind_i_Y;
float wind_i_max;

void StabilizationClass::setMaxAngels() {

	wind_i_max = Balance.get_max_angle() * 0.42f;
	pid_hor.set_kI_max(Balance.get_max_angle());
	pid_hor.hi_2_error_max_diff(Balance.get_max_angle());
}
void StabilizationClass::setMinMaxI_Thr() {
	pid_ver.imax(Balance.get_min_throttle()*(float)cos(Balance.get_max_angle()*GRAD2RAD)-HOVER_THROTHLE, Balance.get_max_throttle() - HOVER_THROTHLE);
}
void StabilizationClass::to_max_ang(const float ang, float& angX, float& angY) {
	float k = sqrt(angX * angX + angY * angY);
	if (k > ang) {
		k = ang / k;
		angX *= k;
		angY *= k;
	}
}
void StabilizationClass::init(){

	dist2speed_XY =  0.2f;
	pid_hor.kP(3);
	pid_hor.set_kI(1);
	xy_kD = 5;// 1.8;
	setMaxAngels();
	def_max_speedXY=current_max_speed_xy = 10;
	min_stab_XY_speed =  1.3f;
	//-------------------------------------------
	min_stab_Z_speed = 1.3f;
	def_max_speedZ_P = current_max_speedZ_P =  5;
	def_max_speedZ_M = current_max_speedZ_M = -5;
	//--------------------------------------------------------------------------
	alt2speedZ = 0.3f;
	pid_ver.kP( 0.06f );
	pid_ver.kI( 0.015f );
	z_kD=0.024f;
	setMinMaxI_Thr();
	wind_i_X = wind_i_Y = 0;
	windRC = 0.001f;
	
	//----------------------------------------------------------------------------
	y_error=x_error=z_error=0;
}
void StabilizationClass::setDefaultMaxSpeeds4Return2HOME() {
	current_max_speed_xy = def_max_speedXY;
	if (current_max_speed_xy < MIN_SPEED_TO_GO_TO_HOME_XY)
		current_max_speed_xy = MIN_SPEED_TO_GO_TO_HOME_XY;
	current_max_speedZ_P = def_max_speedZ_P;
	if (current_max_speedZ_P < MIN_SPEED_TO_GO_TO_HOME_Z)
		current_max_speedZ_P = MIN_SPEED_TO_GO_TO_HOME_Z;
	current_max_speedZ_M = def_max_speedZ_M;
	if (current_max_speedZ_M > -MIN_SPEED_TO_GO_TO_HOME_Z)
		current_max_speedZ_M = -MIN_SPEED_TO_GO_TO_HOME_Z;
}



void StabilizationClass::setDefaultMaxSpeeds(){//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	current_max_speed_xy = def_max_speedXY;
	current_max_speedZ_P = def_max_speedZ_P;// MAX_VER_SPEED_PLUS;
	current_max_speedZ_M = def_max_speedZ_M;// MAX_VER_SPEED_MINUS;
}


void StabilizationClass::max_speed_limiter(float &x, float &y) {
	const float max_ = max(abs(x), abs(y));

	if (max_ > 0) {
		const float speed2 = (x * x + y * y);
		const float maxSpeed2 = current_max_speed_xy * current_max_speed_xy;
		if (speed2 > maxSpeed2) {
			const float k = (float)sqrt(maxSpeed2 / speed2);
			x *= k;
			y *= k;
		}
	}

}
void StabilizationClass::setNeedPos2Home() {
	needXR = needXV = needYR = needYV = 0;
}
void StabilizationClass::dist2speed(float &x, float &y) {
	x = dist2speed_XY * x;
	y = dist2speed_XY * y;
	max_speed_limiter(x, y);

}
void StabilizationClass::speed2dist(float &x, float &y) {
	max_speed_limiter(x, y);
	x /= dist2speed_XY;
	y /= dist2speed_XY;
}

void StabilizationClass::setNeedPos(float x, float y) {
	needXR=needXV = x;
	needYR=needYV = y;
}


void StabilizationClass::fromLoc2Pos(long lat, long lon, double&x, double&y) {
	GPS.loc.fromLoc2Pos(lat, lon, x, y);
	Mpu.getXYRelative2Zero(x, y);
}
void StabilizationClass::setNeedLoc(long lat, long lon, double&x, double&y) {
	fromLoc2Pos(lat, lon, x, y);
	setNeedPos((float)x, (float)y);
	
}
void StabilizationClass::set_max_speed_hor(float &s, bool only_test) {
	const float _max_speed_xy = (s < min_stab_XY_speed) ? min_stab_XY_speed : ( s > def_max_speedXY ? def_max_speedXY : s);
	s = _max_speed_xy;
	if (!only_test)
		current_max_speed_xy = _max_speed_xy;
}
void StabilizationClass::set_max_sped_ver(float &maxP, float &maxM, bool only_test) {
	float _max_speedZ_P, _max_speedZ_M;
	if (maxP == 0 && maxM == 0) {
		_max_speedZ_P = min_stab_Z_speed;
		_max_speedZ_M = -min_stab_Z_speed;
	}
	else {
		 if (maxP > def_max_speedZ_P)
			_max_speedZ_P = def_max_speedZ_P;
		else
			_max_speedZ_P = maxP;

		if (maxM < def_max_speedZ_M)
			_max_speedZ_M = def_max_speedZ_M;
		else
			_max_speedZ_M = maxM;
	}
	maxP = _max_speedZ_P;
	maxM = _max_speedZ_M;
	if (only_test == false) {
		current_max_speedZ_M = _max_speedZ_M;
		current_max_speedZ_P = _max_speedZ_P;
	}
}
void StabilizationClass::add2NeedPos(float speedX, float speedY, float dt) {
	current_max_speed_xy = (speedX == 0 && speedY == 0) ? min_stab_XY_speed : sqrt(speedX * speedX + speedY * speedY);
	if (current_max_speed_xy > def_max_speedXY)
		current_max_speed_xy = def_max_speedXY;

	static bool f_stop_x = false;
	static bool f_stop_y = false;
	if (speedX == 0) {
		if (f_stop_x == false) {
			f_stop_x = true;
			needXR = needXV = (float)Mpu.get_Est_X();
		}
	}
	else {
		if (f_stop_x) {
			needXR = needXV = (float)Mpu.get_Est_X();
			f_stop_x = false;
		}
		float distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needXR += speedX * dt;
		needXV = needXR + distX;
	}
	if (speedY == 0) {
		if (f_stop_y == false) {
			f_stop_y = true;
			needYV = needYR = (float)Mpu.get_Est_Y();
		}
	}
	else {
		if (f_stop_y) {
			f_stop_y = false;
			needYV = needYR = (float)Mpu.get_Est_Y();
		}
		float distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needYR += speedY * dt;
		needYV = needYR + distY;
	}
	
}
float StabilizationClass::get_dist2goal(){
	double dx= Mpu.get_Est_X() - needXV;
	double dy = Mpu.get_Est_Y() - needYV;
	return (float)sqrt(dx*dx + dy * dy);
}
#define _PITCH 0
#define _ROLL 1
void StabilizationClass::calculate_wind_i(float world_ang[]) {
	if (
		Balance.get_throttle()>0.40f &&
		Mpu.get_est_LF_hor_abs_speed() < 1.3f && 
		Mpu.get_est_LF_hor_abs_acc() < 0.5f && 
		Mpu.get_est_LF_ver_abs_speed() < 1.3f && 
		Mpu.get_est_LF_ver_abs_acc() < 0.5f) 
	{
		const float owx = wind_i_X;
		const float owy = wind_i_Y;
		wind_i_X += (world_ang[_PITCH]) * windRC;
		wind_i_Y += (world_ang[_ROLL]) * windRC;
		to_max_ang(wind_i_max, wind_i_X, wind_i_Y);
		world_ang[_PITCH] -= (wind_i_X - owx);
		world_ang[_ROLL] -= (wind_i_Y - owy);
		//Debug.dump(wind_i_X, wind_i_Y, 0, 0);
		const float wind_i = sqrt(wind_i_X * wind_i_X + wind_i_Y * wind_i_Y);
		pid_hor.set_kI_max(Balance.get_max_angle() - wind_i);
		pid_hor.hi_2_error_max_diff(Balance.get_max_angle() - wind_i);

		
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::XY(float &pitch, float&roll){//dont work 
		float need_speedX, need_speedY;
		float tx, ty;
		if (Autopilot.progState() && Prog.intersactionFlag) {
			need_speedX = (float)Prog.need_speedX;
			need_speedY = (float)Prog.need_speedY;
		}
		else {
			tx = (float)Mpu.get_Est_X();
			need_speedX = (needXV - tx);
			ty = (float)Mpu.get_Est_Y();
			need_speedY = (needYV - ty);
			
		}// a=v*v/(2s)

		dist2speed(need_speedX, need_speedY);

		x_error += (((float)Mpu.get_Est_SpeedX() - need_speedX) - x_error) * XY_FILTER;
		y_error += (((float)Mpu.get_Est_SpeedY() - need_speedY) - y_error) * XY_FILTER;

		float *world_ang = pid_hor.get_pid(x_error, y_error, Mpu.get_dt());
		
		calculate_wind_i(world_ang);
		//Debug.dump(wind_i_X,wind_i_Y, pid_hor.get_kI_max(), 1+Mpu.get_est_LF_ver_acc()/9.8f);
		const float mkdX = (x_error == 0)?1:(1 / (abs(x_error)*5));
		const float mkdY = (y_error == 0)?1:(1 / (abs(y_error)*5));
		///Debug.dump(fmin(1, mkdX), fmin(1, mkdY), x_error, y_error);
		world_ang[_PITCH] += ((float)Mpu.get_Est_accX() * xy_kD) * fmin(1.0f, mkdX) + wind_i_X;
		world_ang[_ROLL] += ((float)Mpu.get_Est_accY() * xy_kD) * fmin(1.0f, mkdY) + wind_i_Y;
		//----------------------------------------------------------------from world to local X Y
		float acczK = 1 + (float)Mpu.get_Est_accZ() / 9.8f;
		acczK = constrain(acczK, 0.7f, 1.6f);
		pitch = (-(float)Mpu.cosYaw * world_ang[_PITCH] - Mpu.sinYaw * world_ang[_ROLL]) / acczK;
		roll = ((float)Mpu.cosYaw * world_ang[_ROLL] - Mpu.sinYaw * world_ang[_PITCH]) / acczK;


}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float StabilizationClass::Z(){


	//-------------stab

	const float need_speedZ = getSpeed_Z(Autopilot.fly_at_altitude() - (float)Mpu.get_Est_Alt());
	z_error += ( (need_speedZ - Mpu.get_Est_SpeedZ()) - z_error) * Z_FILTER;

	//mc_z += (need_accZ - Mpu.get_Est_accZ() - mc_z) * Z_FILTER;

	float fZ = (HOVER_THROTHLE + pid_ver.get_pid(z_error, Mpu.get_dt()) - Mpu.get_Est_accZ() * z_kD);// *Balance.powerK();

	//Debug.dump(Mpu.get_Est_accZ(), fZ, Mpu.get_Est_SpeedZ(), need_speedZ);
	//mc_z += (need_speedZ - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	//float fZ = HOVER_THROTHLE +  pids[SPEED_Z_PID].get_pid(mc_z, Mpu.get_dt())*Balance.powerK();
	return fZ;
}

void StabilizationClass::resset_z(){
	z_error = 0;
	pid_ver.reset_I();
	pid_ver.set_integrator(fmax(HOVER_THROTHLE,Autopilot.get_throttle()) - HOVER_THROTHLE);
	
}
void StabilizationClass::resset_xy_integrator(){
	x_error = y_error = 0;
	pid_hor.reset_integrators();
}


// {"alt to speed","speed to acc","acc_2_power","max acc","SPEED_KP","SPEED_I","MAX_SPEED_P","MAX_SPEED_M","FILTR",_null},
string StabilizationClass::get_z_set() {

	ostringstream convert;
	convert << \
		alt2speedZ << "," << \
		pid_ver.kP() << "," << \
		pid_ver.kI() << "," << \
		z_kD << "," << \
		def_max_speedZ_P << "," << \
		def_max_speedZ_M << "," << \
		min_stab_Z_speed << "," << \
		Z_FILTER;
	string ret = convert.str();
	return string(ret);

}
void StabilizationClass::setZ(const float  *ar) {

	uint8_t i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK) {
		float t;
		Settings.set(ar[i++], alt2speedZ);
		t = pid_ver.kP();
		Settings.set(ar[i++], t);
		pid_ver.kP(t);
		t = pid_ver.kI();
		Settings.set(ar[i++], t);
		pid_ver.kI(t);
		Settings.set(ar[i++], z_kD);
		Settings.set(ar[i++], def_max_speedZ_P);
		def_max_speedZ_P = constrain(def_max_speedZ_P, 1, 20);
		Settings.set(ar[i++], def_max_speedZ_M);
		def_max_speedZ_M = constrain(def_max_speedZ_M, -10, -1);
		Autopilot.set_sensZ(def_max_speedZ_P, def_max_speedZ_M);

		min_stab_Z_speed = ar[i++];
		min_stab_Z_speed = constrain(min_stab_Z_speed, 1.3f, def_max_speedZ_P);



		Settings.set(ar[i++], Z_FILTER);
		if (Z_FILTER > 1)
			Z_FILTER = 1;
		else if (Z_FILTER < 0.01f)
			Z_FILTER = 0.01f;
	}

	cout << "Stabilization Z set:\n";
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;
}



//{"dist to speed", "speed to acc", "acc_2_angle", "max acc", "SPEED_KP", "SPEED_I", "max_speed", "FILTR", _null, _null},
string StabilizationClass::get_xy_set() {
	ostringstream convert;
	convert << \
		dist2speed_XY << "," << \
		pid_hor.kP() << "," << \
		pid_hor.get_kI() << "," << \
		windRC <<","<<\
		xy_kD <<","<<\
		def_max_speedXY << "," << \
		min_stab_XY_speed << "," << \
		XY_FILTER;

	string ret = convert.str();
	return string(ret);
}
void StabilizationClass::setXY(const float  *ar){
	uint8_t i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		float t;
		Settings.set(ar[i++], dist2speed_XY);
		t = pid_hor.kP();
		Settings.set(ar[i++], t);
		pid_hor.kP(t);
		t = pid_hor.get_kI();
		Settings.set(ar[i++], t);
		pid_hor.set_kI(t);
		Settings.set(ar[i++], windRC);
		Settings.set(ar[i++], xy_kD);
		Settings.set(ar[i++], def_max_speedXY);
		def_max_speedXY = constrain(def_max_speedXY, 1, 15);
		Autopilot.set_sensXY(def_max_speedXY);
		min_stab_XY_speed = ar[i++];
		if (min_stab_XY_speed < 1.3)min_stab_XY_speed = 1.3f;
		if (min_stab_XY_speed > def_max_speedXY)min_stab_XY_speed = def_max_speedXY;
		Settings.set(ar[i++], XY_FILTER);
		if (XY_FILTER > 1)
			XY_FILTER = 1;
		else if (XY_FILTER < 0.01f)
			XY_FILTER = 0.01f;

	}
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;

}

StabilizationClass Stabilization;
