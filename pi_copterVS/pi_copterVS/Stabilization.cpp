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
	allowance = 2;
	dist2speed_XY = 0.5f;// 0.2f;
	pid_hor.kP(7);
	pid_hor.set_kI(1);
	pid_hor.set_kI_max(15);
	xy_kD = 5;
	def_max_speedXY=current_max_speed_xy = 10;
	min_stab_XY_speed =  10;
	//-------------------------------------------
	min_stab_Z_speed = 3;
	def_max_speedZ_P = current_max_speedZ_P =  5;
	def_max_speedZ_M = current_max_speedZ_M = -5;
	//--------------------------------------------------------------------------
	alt2speedZ = 0.3f;
	pid_ver.kP( 0.06f );
	pid_ver.kI( 0.015f );
	z_kD=0.024f;
	setMinMaxI_Thr();

	
	//----------------------------------------------------------------------------
	z_error=0;
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
		const float maxSpeed2 = (current_max_speed_xy + allowance) * (current_max_speed_xy + allowance);
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
			needXR = needXV;// = (float)GPS.loc.dX;
		}
	}
	else {
		if (f_stop_x) {
			needXR = needXV;// = (float)GPS.loc.dX;
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
			needYV = needYR = (float)GPS.loc.dY;
		}
	}
	else {
		if (f_stop_y) {
			f_stop_y = false;
			needYV = needYR = (float)GPS.loc.dY;
		}
		float distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needYR += speedY * dt;
		needYV = needYR + distY;
	}
	
}
float StabilizationClass::get_dist2goal(){
	double dx= GPS.loc.dX - needXV;
	double dy = GPS.loc.dY - needYV;
	return (float)sqrt(dx*dx + dy * dy);
}

#define _PITCH 0
#define _ROLL 1

float hor_pos_k = 0.5f;
float hor_speed_k = 2;
void StabilizationClass::Hor_position(float& pitch, float& roll) {
	float pos_angX = (Mpu.get_Est_X_() - needXV) * hor_pos_k;
	pos_angX *= abs(pos_angX);

	float pos_angY = (Mpu.get_Est_Y_() - needYV)* hor_pos_k;
	pos_angY *= abs(pos_angY);
	to_max_ang(35, pos_angX, pos_angY);

	float speed_angX = Mpu.get_Est_SpeedX_() * hor_speed_k;
	speed_angX *= abs(speed_angX);

	float speed_angY = Mpu.get_Est_SpeedY_() * hor_speed_k;
	speed_angY *= abs(speed_angY);

	to_max_ang(35, speed_angX, speed_angY);

	Debug.dump(pos_angX, speed_angX, pos_angY, speed_angY);
	pos_angX += speed_angX;
	pos_angY += speed_angY;


	pitch = (-(float)Mpu.cosYaw * pos_angX - Mpu.sinYaw * pos_angY);
	roll = ((float)Mpu.cosYaw * pos_angY - Mpu.sinYaw * pos_angX);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::Hor_speed(float &pitch, float&roll){//dont work 
		float need_speedX, need_speedY;
		float tx, ty;
		if (Autopilot.progState() && Prog.intersactionFlag) {
			need_speedX = (float)Prog.need_X;
			need_speedY = (float)Prog.need_Y;
		}
		else {

			tx = (float)GPS.loc.dX;
			need_speedX = (needXV - tx);
			ty = (float)GPS.loc.dY;
			need_speedY = (needYV - ty);
			
		}// a=v*v/(2s)





		if (Commander.getPitch() == 0 && Commander.getRoll() == 0)
			return Hor_position(pitch, roll);






		dist2speed(need_speedX, need_speedY);
	
	//	const float need_direction=atan2(need_speedY, need_speedX);


		const float x_error = (float)(GPS.loc.speedX - need_speedX);
		const float y_error = (float)(GPS.loc.speedY - need_speedY);

		float *world_ang = pid_hor.get_pid(x_error, y_error, Mpu.get_dt());

		///Debug.dump(fmin(1, mkdX), fmin(1, mkdY), x_error, y_error);
		world_ang[_PITCH] += (float)Mpu.w_accX * xy_kD;
		world_ang[_ROLL] += (float)Mpu.w_accY * xy_kD;
		//----------------------------------------------------------------from world to local X Y
		pitch = (-(float)Mpu.cosYaw * world_ang[_PITCH] - Mpu.sinYaw * world_ang[_ROLL]);
		roll = ((float)Mpu.cosYaw * world_ang[_ROLL] - Mpu.sinYaw * world_ang[_PITCH]);


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
		pid_hor.get_kI() <<","<<\
		pid_hor.get_kI_max()<<","<<\
		xy_kD << "," << \
		def_max_speedXY << "," << \
		min_stab_XY_speed << "," << \
		allowance;


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

		float temp = pid_hor.get_kI();
		Settings.set(ar[i++], temp);
		pid_hor.set_kI(temp);

		temp = ar[i++];
		constrain(temp, 0, 15);
		pid_hor.set_kI_max(temp);

		Settings.set(ar[i++], xy_kD);
		def_max_speedXY= ar[i++];
		def_max_speedXY = constrain(def_max_speedXY, 3, 10);
		Autopilot.set_sensXY(def_max_speedXY);
		min_stab_XY_speed = ar[i++];
		min_stab_XY_speed = constrain(min_stab_XY_speed, 1.3, def_max_speedXY);
		Settings.set(ar[i++], allowance);
		allowance = constrain(allowance, 0, 4);

	}
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;

}

StabilizationClass Stabilization;
