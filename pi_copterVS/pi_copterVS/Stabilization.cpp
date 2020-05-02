// 
// 
// 

#include "define.h"
#include "WProgram.h"
#include "Stabilization.h"
#include "mpu.h"
#include "AP_PID.h"
#include "mpu.h"
#include "MS5611.h"
#include "Autopilot.h"

#include "GPS.h"
#include "debug.h"
#include "Balance.h"
#include "Prog.h"
#include "Log.h"
#include "Settings.h"


float Z_FILTER = 0.33;
float XY_FILTER = 0.33;
void StabilizationClass::setMaxAng() {
	set_acc_xy_speed_imax(Balance.get_max_angle());
}
void StabilizationClass::setMinMaxI_Thr() {
	pids[SPEED_Z_PID].imax(Balance.get_min_throttle()*cos(Balance.get_max_angle()*GRAD2RAD)-HOVER_THROTHLE, Balance.get_max_throttle() - HOVER_THROTHLE);
}
void StabilizationClass::init(){

	dist2speed_XY = 0.2f;//0.5 
	set_acc_xy_speed_kp(6);
	set_acc_xy_speed_kI(3);
	set_acc_xy_speed_imax(Balance.get_max_angle());

	def_max_speedXY=max_speed_xy = 10;
	min_stab_hor_speed = 0.5;
	min_stab_ver_speed = 1;
	def_max_speedZ_P = max_speedZ_P =  3;
	def_max_speedZ_M = max_speedZ_M = -3;
	//--------------------------------------------------------------------------

	alt2speedZ = 0.2;
	pids[SPEED_Z_PID].kP( 0.05 );
	pids[SPEED_Z_PID].kI( 0.025 );

	setMinMaxI_Thr();
	
	//----------------------------------------------------------------------------
	mc_x=mc_y=mc_z=0;
}
void StabilizationClass::setDefaultMaxSpeeds4Return2HOME() {
	max_speed_xy = def_max_speedXY;
	if (max_speed_xy < MIN_SPEED_TO_GO_TO_HOME_XY)
		max_speed_xy = MIN_SPEED_TO_GO_TO_HOME_XY;
	max_speedZ_P = def_max_speedZ_P;
	if (max_speedZ_P < MIN_SPEED_TO_GO_TO_HOME_Z)
		max_speedZ_P = MIN_SPEED_TO_GO_TO_HOME_Z;
	max_speedZ_M = def_max_speedZ_M;
	if (max_speedZ_M > -MIN_SPEED_TO_GO_TO_HOME_Z)
		max_speedZ_M = -MIN_SPEED_TO_GO_TO_HOME_Z;
}



void StabilizationClass::setDefaultMaxSpeeds(){//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	max_speed_xy = def_max_speedXY;
	max_speedZ_P = def_max_speedZ_P;// MAX_VER_SPEED_PLUS;
	max_speedZ_M = def_max_speedZ_M;// MAX_VER_SPEED_MINUS;
}


void StabilizationClass::max_speed_limiter(float &x, float &y) {
	const float max_ = max(abs(x), abs(y));
	if (max_ > 0) {
		x = x / max_ * max_speed_xy;
		y = y / max_ * max_speed_xy;
		const float speed2 = (x * x + y * y);
		const float maxSpeed2 = max_speed_xy * max_speed_xy;
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
	setNeedPos(x, y);
	
}
void StabilizationClass::set_max_speed_hor(float &s, bool only_test) {
	const float _max_speed_xy = (s < min_stab_hor_speed) ? min_stab_hor_speed : ( s > def_max_speedXY ? def_max_speedXY : s);
	s = _max_speed_xy;
	if (!only_test)
		max_speed_xy = _max_speed_xy;
}
void StabilizationClass::set_max_sped_ver(float &maxP, float &maxM, bool only_test) {
	float _max_speedZ_P, _max_speedZ_M;
	if (maxP == 0 && maxM == 0) {
		_max_speedZ_P = min_stab_ver_speed;
		_max_speedZ_M = -min_stab_ver_speed;
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
		max_speedZ_M = _max_speedZ_M;
		max_speedZ_P = _max_speedZ_P;
	}
}
void StabilizationClass::add2NeedPos(float speedX, float speedY, float dt) {


	max_speed_xy = (speedX == 0 && speedY == 0) ? min_stab_hor_speed : sqrt(speedX * speedX + speedY * speedY);
	if (max_speed_xy > def_max_speedXY)
		max_speed_xy = def_max_speedXY;

	static bool f_stop_x = false;
	static bool f_stop_y = false;
	if (speedX == 0) {
		if (f_stop_x == false) {
			f_stop_x = true;
			needXR = needXV = Mpu.get_Est_X();
		}
	}
	else {
		if (f_stop_x) {
			needXR = needXV = Mpu.get_Est_X();
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
			needYV = needYR = Mpu.get_Est_Y();
		}
	}
	else {
		if (f_stop_y) {
			f_stop_y = false;
			needYV = needYR = Mpu.get_Est_Y();
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::XY(float &pitch, float&roll){//dont work 
		float need_speedX, need_speedY;
		float tx, ty;
		if (Autopilot.progState() && Prog.intersactionFlag) {
			need_speedX = Prog.need_speedX;
			need_speedY = Prog.need_speedY;
		}
		else {
			tx = Mpu.get_Est_X();
			need_speedX = (needXV - tx);
			ty = Mpu.get_Est_Y();
			need_speedY = (needYV - ty);
			
		}// a=v*v/(2s)
		dist2speed(need_speedX, need_speedY);





		const float need_accX = (Mpu.get_Est_SpeedX() - need_speedX) * 0.5;
		const float need_accY = (Mpu.get_Est_SpeedY() - need_speedY) * 0.5;

		mc_x += (need_accX - Mpu.get_Est_accX() - mc_x) * XY_FILTER;
		const float w_pitch = -(pids[SPEED_X_SPEED].get_pid(mc_x, Mpu.get_dt()));
		mc_y += (need_accY - Mpu.get_Est_accY() - mc_y) * XY_FILTER;
		const float w_roll = pids[SPEED_Y_SPEED].get_pid(mc_y, Mpu.get_dt());

		
	
		//----------------------------------------------------------------преобр. в относительную систему координат
		pitch = (float)(Mpu.cosYaw*w_pitch - Mpu.sinYaw*w_roll);
		roll = (float)(Mpu.cosYaw*w_roll + Mpu.sinYaw*w_pitch);


		//Debug.dump(Mpu.get_Est_accX(), Mpu.get_Est_accY(), pitch, roll);
		//Debug.load(0, pitch, roll);
		//Debug.dump();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float StabilizationClass::Z(){


	//-------------stab

	const float need_speedZ = getSpeed_Z(Autopilot.fly_at_altitude() - Mpu.get_Est_Alt());
	const float need_accZ = (need_speedZ - Mpu.get_Est_SpeedZ()) * 1;
	mc_z += (need_accZ - Mpu.get_Est_accZ() - mc_z) * Z_FILTER;
	float fZ = HOVER_THROTHLE + pids[SPEED_Z_PID].get_pid(mc_z, Mpu.get_dt()) * Balance.powerK();


	//mc_z += (need_speedZ - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	//float fZ = HOVER_THROTHLE +  pids[SPEED_Z_PID].get_pid(mc_z, Mpu.get_dt())*Balance.powerK();
	return fZ;
}

void StabilizationClass::resset_z(){
	mc_z = 0;
	pids[SPEED_Z_PID].reset_I();
	pids[SPEED_Z_PID].set_integrator(fmax(HOVER_THROTHLE,Autopilot.get_throttle()) - HOVER_THROTHLE);
	
}
void StabilizationClass::resset_xy_integrator(){
	mc_x = mc_y = 0;
	pids[SPEED_X_SPEED].reset_I();
	pids[SPEED_Y_SPEED].reset_I();
}









// {"alt to speed","speed to acc","acc_2_power","max acc","SPEED_KP","SPEED_I","MAX_SPEED_P","MAX_SPEED_M","FILTR",_null},
string StabilizationClass::get_z_set() {

	ostringstream convert;
	convert << \
		alt2speedZ << "," << \
		pids[SPEED_Z_PID].kP() << "," << \
		pids[SPEED_Z_PID].kI() << "," << \
		def_max_speedZ_P << "," << \
		def_max_speedZ_M << "," << \
		min_stab_ver_speed << "," << \
		Z_FILTER;
	string ret = convert.str();
	return string(ret);

}
void StabilizationClass::setZ(const float  *ar) {

	uint8_t i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK) {
		float t;
		Settings.set(ar[i++], alt2speedZ);
		t = pids[SPEED_Z_PID].kP();
		Settings.set(ar[i++], t);
		pids[SPEED_Z_PID].kP(t);
		t = pids[SPEED_Z_PID].kI();
		Settings.set(ar[i++], t);
		pids[SPEED_Z_PID].kI(t);
		Settings.set(ar[i++], def_max_speedZ_P);
		def_max_speedZ_P = constrain(def_max_speedZ_P, 1, 20);
		Settings.set(ar[i++], def_max_speedZ_M);
		def_max_speedZ_M = constrain(def_max_speedZ_M, -10, -1);
		Autopilot.set_sensZ(def_max_speedZ_P, def_max_speedZ_M);


		Settings.set(ar[i++], min_stab_ver_speed);
		Settings.set(ar[i++], Z_FILTER);
		if (Z_FILTER > 1)
			Z_FILTER = 1;
		else if (Z_FILTER < 0.01)
			Z_FILTER = 0.01;
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
		pids[SPEED_X_SPEED].kP() << "," << \
		pids[SPEED_X_SPEED].kI() << "," << \
		def_max_speedXY << "," << \
		min_stab_hor_speed << "," << \
		XY_FILTER;

	string ret = convert.str();
	return string(ret);
}
void StabilizationClass::setXY(const float  *ar){
	uint8_t i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		float t;
		Settings.set(ar[i++], dist2speed_XY);
		t = pids[SPEED_X_SPEED].kP();
		Settings.set(ar[i++], t);
		set_acc_xy_speed_kp(t);
		t = pids[SPEED_X_SPEED].kI();
		Settings.set(ar[i++], t);
		set_acc_xy_speed_kI(t);
		Settings.set(ar[i++], def_max_speedXY);
		def_max_speedXY = constrain(def_max_speedXY, 1, 20);
		Autopilot.set_sensXY(def_max_speedXY);
		Settings.set(ar[i++], min_stab_hor_speed);
		Settings.set(ar[i++], XY_FILTER);
		if (XY_FILTER > 1)
			XY_FILTER = 1;
		else if (XY_FILTER < 0.01)
			XY_FILTER = 0.01;

	}
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;

}

StabilizationClass Stabilization;
