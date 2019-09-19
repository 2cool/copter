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
	pids[SPEED_Z_PID].imax(Balance.get_min_throttle()-HOVER_THROTHLE, Balance.get_max_throttle() - HOVER_THROTHLE);
}
//"dist to speed","speed to acc","SPEED_KP","SPEED_I","SPEED_imax","max_speed","FILTR"
void StabilizationClass::init(){

	dist2speed_XY = 0.2f;//0.5
	set_acc_xy_speed_kp(5);
	set_acc_xy_speed_kI(2.5);
	set_acc_xy_speed_imax(Balance.get_max_angle());
	max_speed_xy = MAX_HOR_SPEED;
	//--------------------------------------------------------------------------

	alt2speedZ = 0.2;
	pids[SPEED_Z_PID].kP( 0.18 );
	pids[SPEED_Z_PID].kI( 0.25 );

	setMinMaxI_Thr();
	max_speedZ_P =  MAX_VER_SPEED_PLUS;
	max_speedZ_M = MAX_VER_SPEED_MINUS;
	//----------------------------------------------------------------------------
	d_speedX=d_speedY=mc_z=0;
}
//bool flx = false, fly = false;

double StabilizationClass::accxy_stab(double dist, double maxA, double timeL) {
	
	return sqrt(2 * maxA*dist) - maxA*timeL;

}
double StabilizationClass::accxy_stab_rep(double speed, double maxA, double timeL) {
	double t = speed / maxA + timeL;
	return 0.5*maxA*t*t;
}

void StabilizationClass::setDefaultMaxSpeeds(){//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	max_speed_xy = MAX_HOR_SPEED;
	max_speedZ_P = MAX_VER_SPEED_PLUS;
	max_speedZ_M = MAX_VER_SPEED_MINUS;
}


void StabilizationClass::max_speed_limiter(double &x, double &y) {
	const double speed2 = (x*x + y * y);
	const double maxSpeed2 = max_speed_xy * max_speed_xy;
	if (speed2 > maxSpeed2) {
		const double k = (double)sqrt(maxSpeed2 / speed2);
		x *= k;
		y *= k;
	}
}
void StabilizationClass::setNeedPos2Home() {
	needXR = needXV = needYR = needYV = 0;
}
void StabilizationClass::dist2speed(double &x, double &y) {
	x = dist2speed_XY * x;
	y = dist2speed_XY * y;
	max_speed_limiter(x, y);

}
void StabilizationClass::speed2dist(double &x, double &y) {
	max_speed_limiter(x, y);
	x /= dist2speed_XY;
	y /= dist2speed_XY;
}

void StabilizationClass::setNeedPos(double x, double y) {
	needXR=needXV = x;
	needYR=needYV = y;
}


void StabilizationClass::fromLoc2Pos(long lat, long lon, double &x, double &y) {
	
	GPS.loc.fromLoc2Pos(lat, lon, x, y);
	Mpu.getXYRelative2Zero(x, y);
}
void StabilizationClass::setNeedLoc(long lat, long lon, double &x, double &y) {
	fromLoc2Pos(lat, lon, x, y);
	setNeedPos(x, y);
	
}


void StabilizationClass::add2NeedPos(double speedX, double speedY, double dt) {
	static bool flagzx = false;
	static bool flagzy = false;
	if (speedX == 0) {
		if (flagzx == false) {
			flagzx = true;
			needXR = needXV = Mpu.get_Est_X();
		}
	}
	else {
		flagzx = false;
		double distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needXR += speedX * dt;
		needXV = needXR + distX;
	}
	if (speedY == 0) {
		if (flagzy == false) {
			flagzy = true;
			needYV = needYR = Mpu.get_Est_Y();
		}
	}
	else {
		flagzy = false;
		double distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needYR += speedY * dt;
		needYV = needYR + distY;
	}
}
double StabilizationClass::get_dist2goal(){
	double dx= Mpu.get_Est_X() - needXV;
	double dy = Mpu.get_Est_Y() - needYV;
	return sqrt(dx*dx + dy * dy);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//double old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::XY(float &pitch, float&roll){//dont work 
		double need_speedX, need_speedY;
		double tx, ty;
		if (Autopilot.progState() && Prog.intersactionFlag) {
			need_speedX = -Prog.need_speedX;
			need_speedY = -Prog.need_speedY;
		}
		else {
			tx = Mpu.get_Est_X(); 
			need_speedX = (tx-needXV);
			ty = Mpu.get_Est_Y();
			need_speedY = (ty-needYV);
			
		}//вичислять нужное ускорение по форумуле a=v*v/(2s)
		dist2speed(need_speedX, need_speedY);
		d_speedX += ((need_speedX + Mpu.get_Est_SpeedX()) - d_speedX)*XY_FILTER;
		d_speedY += ((need_speedY + Mpu.get_Est_SpeedY()) - d_speedY)*XY_FILTER;

	//	mc_x += (mc_pitch - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	//	mc_z = constrain(mc_z, -max_acc_z, max_acc_z);
	//	const double accX_C =  ((d_speedX * speed_2_acc_XY) - Mpu.w_accX)*acc_2_angle;
	//	const double accY_C = ((d_speedY * speed_2_acc_XY) - Mpu.w_accY)*acc_2_angle;

		const double w_pitch = -(pids[SPEED_X_SPEED].get_pid(d_speedX, Mpu.get_dt()));
		const double w_roll = pids[SPEED_Y_SPEED].get_pid(d_speedY, Mpu.get_dt());

		

		//----------------------------------------------------------------преобр. в относительную систему координат
		pitch = (float)(Mpu.cosYaw*w_pitch - Mpu.sinYaw*w_roll);
		roll = (float)(Mpu.cosYaw*w_roll + Mpu.sinYaw*w_pitch);

		//Debug.load(0, pitch, roll);
		//Debug.dump();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double StabilizationClass::Z(){


	//-------------stab

	const double need_speedZ = getSpeed_Z(Autopilot.fly_at_altitude() - Mpu.get_Est_Alt());

	mc_z += (need_speedZ - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	//const double accZ_C = ((mc_z * speed_2_acc_Z) - Mpu.faccZ)*acc_2_power;



	double fZ = HOVER_THROTHLE +  pids[SPEED_Z_PID].get_pid(mc_z, Mpu.get_dt())*Balance.powerK();
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
		max_speedZ_P << "," << \
		max_speedZ_M << "," << \
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
		Settings.set(ar[i++], max_speedZ_P);
		Settings.set(ar[i++], max_speedZ_M);
		Settings.set(ar[i++], Z_FILTER);
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
		max_speed_xy << "," << \
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
		Settings.set(ar[i++], max_speed_xy);
		Settings.set(ar[i++], XY_FILTER);

	}
	for (uint8_t ii = 0; ii < i; ii++) {
		cout << ar[ii] << ",";
	}
	cout << endl;

}

StabilizationClass Stabilization;
