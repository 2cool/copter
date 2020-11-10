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

	max_wind_ang =10;
	hor_pos_kp = 0.3f;
	hor_speed_kd = 1;
	hor_speed_kp = 7;
	hor_acc_kd = 5;
	def_max_speedXY=current_max_speed_xy = 10;
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
void StabilizationClass::dist2speed(const  float dx, const float dy, float &speed_x, float &speed_y) {
	speed_x = dist2speed_XY * dx;
	speed_y = dist2speed_XY * dy;
	max_speed_limiter(speed_x, speed_y);

}
void StabilizationClass::speed2dist(const float speed_x, const  float speed_y, float &dx, float &dy) {
	dx = speed_x;
	dy = speed_y;
	max_speed_limiter(dx, dy);
	dx /= dist2speed_XY;
	dy /= dist2speed_XY;
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
	const float _max_speed_xy =(s > def_max_speedXY ? def_max_speedXY : s);
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

	current_max_speed_xy = def_max_speedXY;
	static bool f_stop_x = false;
	static bool f_stop_y = false;
	if (speedX == 0) {
		if (f_stop_x == false) {
			f_stop_x = true;
			needXR = needXV;
		}
	}
	else {
		if (f_stop_x) {
			needXR = needXV;
			f_stop_x = false;
		}
		float distX , distY;
		speed2dist(speedX, speedY, distX, distY);
		needXR += speedX * dt;
		needXV = needXR + distX;
	}
	if (speedY == 0) {
		if (f_stop_y == false) {
			f_stop_y = true;
			needYR = needYV;
		}
	}
	else {
		if (f_stop_y) {
			f_stop_y = false;
			needYR = needYV;
		}
		float distX , distY;
		speed2dist(speedX, speedY, distX, distY);
		needYR += speedY * dt;
		needYV = needYR + distY;
	}
	
}
float StabilizationClass::get_dist2goal(){
	double dx= Mpu.get_Est_X_() - needXV;
	double dy = Mpu.get_Est_Y_() - needYV;
	return (float)sqrt(dx*dx + dy * dy);
}

#define _PITCH 0
#define _ROLL 1

static float wind_ang_x = 0, wind_ang_y = 0;
void StabilizationClass::Hor_position(float& pitch, float& roll) {
	
	static float pos_angX = 0, pos_angY = 0;

	const float errorx = (Mpu.get_Est_X_() - needXV);
	const float errory = (Mpu.get_Est_Y_() - needYV);

	

	if (Mpu.get_est_LF_hor_abs_speed() < 0.1) {
		wind_ang_x += (pos_angX - wind_ang_x) * 0.001;
		wind_ang_y += (pos_angY - wind_ang_y) * 0.001;
		to_max_ang(max_wind_ang, wind_ang_x, wind_ang_y);
	//	Debug.dump(wind_ang_x, wind_ang_y, errorx, errory);
	}



	pos_angX = errorx * hor_pos_kp;
	pos_angY = errory * hor_pos_kp;

	pos_angX += wind_ang_x;
	pos_angY += wind_ang_y;

	//to_max_ang(Balance.get_max_angle(), pos_angX, pos_angY);

	float speed_angX = Mpu.get_Est_SpeedX_() * hor_speed_kd;
	speed_angX *= abs(speed_angX);

	float speed_angY = Mpu.get_Est_SpeedY_() * hor_speed_kd;
	speed_angY *= abs(speed_angY);

	//to_max_ang(Balance.get_max_angle(), speed_angX, speed_angY);

//	Debug.dump(pos_angX, speed_angX, pos_angY, speed_angY);
	pos_angX += speed_angX;
	pos_angY += speed_angY;

	pitch = (-(float)Mpu.cosYaw * pos_angX - Mpu.sinYaw * pos_angY);
	roll = ((float)Mpu.cosYaw * pos_angY - Mpu.sinYaw * pos_angX);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::Hor_speed(float &pitch, float&roll){//dont work 
		float dX,dY, need_speedX, need_speedY;

		if (Autopilot.progState() && Prog.intersactionFlag) {
			dX = (float)Prog.need_X;
			dY = (float)Prog.need_Y;
		}
		else {
			dX = (needXV - (float)Mpu.get_Est_X_());
			dY = (needYV - (float)Mpu.get_Est_Y_());
			
		}// a=v*v/(2s)

		if (
			needXR == needXV && needYR == needYV &&
			!Autopilot.progState() &&
			(!Autopilot.go2homeState() || (abs(dX) < 10 && abs(dY) < 10))
			) {
			return Hor_position(pitch, roll);
		}
		else {

			dist2speed(dX, dY, need_speedX, need_speedY);

			const float x_error = (float)(Mpu.get_Est_SpeedX_() - need_speedX);
			const float y_error = (float)(Mpu.get_Est_SpeedY_() - need_speedY);

			float speedX = x_error * hor_speed_kp;
			float speedY = y_error * hor_speed_kp;

			float accX = (float)Mpu.w_accX * hor_acc_kd;
			float accY = (float)Mpu.w_accY * hor_acc_kd;

			speedX += accX+wind_ang_x;
			speedY += accY+wind_ang_y;


			pitch = (-(float)Mpu.cosYaw * speedX - Mpu.sinYaw * speedY);
			roll = ((float)Mpu.cosYaw * speedY - Mpu.sinYaw * speedX);
		}

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
	wind_ang_x = wind_ang_y = 0;
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
		hor_pos_kp << ","<< \
		hor_speed_kd<<","<<\
		max_wind_ang <<","<<\

		dist2speed_XY << "," << \
		hor_speed_kp << "," << \
		hor_acc_kd << "," << \
		def_max_speedXY << "," << \
		allowance;


	string ret = convert.str();
	return string(ret);
}
void StabilizationClass::setXY(const float  *ar){
	uint8_t i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		Settings.set(ar[i++], hor_pos_kp);
		Settings.set(ar[i++], hor_speed_kd);
		max_wind_ang=ar[i++];
		def_max_speedXY = constrain(max_wind_ang, 0, 20);

		Settings.set(ar[i++], dist2speed_XY);
		Settings.set(ar[i++], hor_speed_kp);
		Settings.set(ar[i++], hor_acc_kd);

		def_max_speedXY=ar[i++];
		def_max_speedXY = constrain(def_max_speedXY, 3, 10);
		Settings.set(ar[i++], allowance);
		allowance = constrain(allowance, 0, 4);
	}
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;

}

StabilizationClass Stabilization;
