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

float Z_FILTER = 1;
float XY_FILTER = 1;
void StabilizationClass::setMaxAng() {
	set_acc_xy_speed_imax(Balance.get_max_angle());
}
void StabilizationClass::setMinMaxI_Thr() {
	pids[SPEED_Z_PID].imax(Balance.get_min_throttle()*cos(Balance.get_max_angle()*GRAD2RAD)-HOVER_THROTHLE, Balance.get_max_throttle() - HOVER_THROTHLE);
}
//"dist to speed","speed to acc","SPEED_KP","SPEED_I","SPEED_imax","max_speed","FILTR"
void StabilizationClass::init(){

	dist2speed_XY = 0.2f;//0.5
	set_acc_xy_speed_kp(6);
	set_acc_xy_speed_kI(2.5);
	set_acc_xy_speed_imax(Balance.get_max_angle());
	def_max_speedXY=max_speed_x_P=max_speed_y_P = MAX_HOR_SPEED;
	max_speed_x_M = max_speed_y_M = -MAX_HOR_SPEED;
	min_stab_XY_speed = MIN_STAB_HOR_SPEED;
	min_stab_Z_speed = 0.1;
	def_max_speedZ_P = max_speedZ_P =  MAX_VER_SPEED_PLUS;
	def_max_speedZ_M = max_speedZ_M = MAX_VER_SPEED_MINUS;
	//--------------------------------------------------------------------------

	alt2speedZ = 0.2;
	pids[SPEED_Z_PID].kP( 0.06 );
	pids[SPEED_Z_PID].kI( 0.12 );

	setMinMaxI_Thr();
	
	//----------------------------------------------------------------------------
	d_speedX=d_speedY=mc_z=0;
}

void StabilizationClass::setDefaultMaxSpeeds(){//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	max_speed_x_P = max_speed_y_P = def_max_speedXY;
	max_speed_x_M = max_speed_y_M = -def_max_speedXY;

	max_speedZ_P = def_max_speedZ_P;
	max_speedZ_M = def_max_speedZ_M;
}


void StabilizationClass::max_speed_limiter(float &x, float &y) {
	double speed2 = (x * x + y * y);
	double maxSpeed2;
	
	if (x >= 0) {
		if (y >= 0) 
			maxSpeed2 = max_speed_x_P * max_speed_x_P + max_speed_y_P * max_speed_y_P;
		else 
			maxSpeed2 = max_speed_x_P * max_speed_x_P + max_speed_y_M * max_speed_y_M;
	}else {
		if (y >= 0) 
			maxSpeed2 = max_speed_x_M * max_speed_x_M + max_speed_y_P * max_speed_y_P;
		else 
			maxSpeed2 = max_speed_x_M * max_speed_x_M + max_speed_y_M * max_speed_y_M;
	}
	if (speed2 > maxSpeed2) {

		const float k = (float)sqrt(maxSpeed2 / speed2);
		x *= k;
		y *= k;
	}
	//Debug.dump(max_speed_x_P, max_speed_x_M, max_speed_y_P, max_speed_y_M);

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
	const float _max_speed_xy = (s < min_stab_XY_speed) ? min_stab_XY_speed : ( s > def_max_speedXY ? def_max_speedXY : s);
	s = _max_speed_xy;
	if (!only_test) {
		max_speed_x_P = max_speed_y_P = _max_speed_xy;
		max_speed_x_M = max_speed_y_M = -_max_speed_xy;
	}
}
void StabilizationClass::set_max_sped_ver(float &maxP, float &maxM, bool only_test) {
	maxP = constrain(maxP, min_stab_Z_speed, def_max_speedZ_P);
	maxM = constrain(maxM, def_max_speedZ_M, -min_stab_Z_speed);
	if (only_test == false) {
		max_speedZ_P = maxP;
		max_speedZ_M = maxM;
	}
}
void StabilizationClass::set_max_speedXY(const float speedX, const float speedY) {
	if (speedX == 0) {
		max_speed_x_P = min_stab_XY_speed;
		max_speed_x_M = -min_stab_XY_speed;
	}
	else if (speedX > 0) {
		max_speed_x_P = (speedX>=min_stab_XY_speed)?speedX: min_stab_XY_speed;
		max_speed_x_M = -min_stab_XY_speed;
	}
	else {
		max_speed_x_M = (speedX <= -min_stab_XY_speed) ? speedX : -min_stab_XY_speed;
		max_speed_x_P = min_stab_XY_speed;
	}
	if (speedY == 0) {
		max_speed_y_P = min_stab_XY_speed;
		max_speed_y_M = -min_stab_XY_speed;
	}
	else if (speedY > 0) {
		max_speed_y_P = (speedY >= min_stab_XY_speed) ? speedY : min_stab_XY_speed;
		max_speed_y_M = -min_stab_XY_speed;
	}
	else {
		max_speed_y_M = (speedY <= -min_stab_XY_speed) ? speedY : -min_stab_XY_speed;
		max_speed_y_P = min_stab_XY_speed;
	}

}


void StabilizationClass::add2NeedPos(float speedX, float speedY, float dt) {
	
	set_max_speedXY(speedX, speedY);

	static bool f_stop_x = false;
	static bool f_stop_y = false;
	if (speedX == 0) {
		if (f_stop_x == false ) {
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
		if (Autopilot.progState()) {
			if (need_speedX >= 0) {
				max_speed_x_P = Prog.get_max_speed_XY();
				max_speed_x_M = -min_stab_XY_speed;
			}
			else {
				max_speed_x_M = -Prog.get_max_speed_XY();
				max_speed_x_P = min_stab_XY_speed;
			}
			if (need_speedY >= 0) {
				max_speed_y_P = Prog.get_max_speed_XY();;
				max_speed_y_M = -min_stab_XY_speed;
			}
			else {
				max_speed_y_M = -Prog.get_max_speed_XY();;
				max_speed_y_P = min_stab_XY_speed;
			}
		}
		if (Autopilot.progState() && Prog.intersactionFlag) {
			need_speedX = Prog.need_speedX;
			need_speedY = Prog.need_speedY;
		}
		else {
			tx = Mpu.get_Est_X(); 
			need_speedX = (needXV-tx);
			ty = Mpu.get_Est_Y();
			need_speedY = (needYV-ty);
			
		}//вичислять нужное ускорение по форумуле a=v*v/(2s)

		dist2speed(need_speedX, need_speedY);
		d_speedX += ((Mpu.get_Est_SpeedX() - need_speedX) - d_speedX)*XY_FILTER;
		d_speedY += ((Mpu.get_Est_SpeedY() - need_speedY) - d_speedY)*XY_FILTER;

	//	mc_x += (mc_pitch - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	//	mc_z = constrain(mc_z, -max_acc_z, max_acc_z);
	//	const float accX_C =  ((d_speedX * speed_2_acc_XY) - Mpu.w_accX)*acc_2_angle;
	//	const float accY_C = ((d_speedY * speed_2_acc_XY) - Mpu.w_accY)*acc_2_angle;

		const float w_pitch = -(pids[SPEED_X_SPEED].get_pid(d_speedX, Mpu.get_dt()));
		const float w_roll = pids[SPEED_Y_SPEED].get_pid(d_speedY, Mpu.get_dt());

		
	//	Debug.dump(max_speed_xy, max_speedZ_P, max_speedZ_M, 0);
		//----------------------------------------------------------------преобр. в относительную систему координат
		pitch = (float)(Mpu.cosYaw*w_pitch - Mpu.sinYaw*w_roll);
		roll = (float)(Mpu.cosYaw*w_roll + Mpu.sinYaw*w_pitch);
		//Debug.load(0, pitch, roll);
		//Debug.dump();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float StabilizationClass::Z(){
	//-------------stab
	const float need_speedZ = getSpeed_Z(Autopilot.fly_at_altitude() - Mpu.get_Est_Alt());
	mc_z += (need_speedZ - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	float fZ = HOVER_THROTHLE +  pids[SPEED_Z_PID].get_pid(mc_z, Mpu.get_dt())*Balance.powerK();
	return fZ;
}

void StabilizationClass::resset_z(){
	mc_z = 0;
	pids[SPEED_Z_PID].reset_I();
	pids[SPEED_Z_PID].set_integrator(fmax(HOVER_THROTHLE,Autopilot.get_throttle()) - HOVER_THROTHLE);
	
}
void StabilizationClass::resset_xy_integrator(){
	d_speedX = d_speedY = 0;
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
		t = pids[SPEED_Z_PID].kP();
		Settings.set(ar[i++], t);
		pids[SPEED_Z_PID].kP(t);
		t = pids[SPEED_Z_PID].kI();
		Settings.set(ar[i++], t);
		pids[SPEED_Z_PID].kI(t);
		Settings.set(ar[i++], def_max_speedZ_P);
		Settings.set(ar[i++], def_max_speedZ_M);
		Settings.set(ar[i++], min_stab_Z_speed);
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
		t = pids[SPEED_X_SPEED].kP();
		Settings.set(ar[i++], t);
			set_acc_xy_speed_kp(t);
		t = pids[SPEED_X_SPEED].kI();
		Settings.set(ar[i++], t);
			set_acc_xy_speed_kI(t);
		Settings.set(ar[i++], def_max_speedXY);
		Settings.set(ar[i++], min_stab_XY_speed);
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
