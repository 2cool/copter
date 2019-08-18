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

float StabilizationClass::accxy_stab(float dist, float maxA, float timeL) {
	
	return sqrt(2 * maxA*dist) - maxA*timeL;

}
float StabilizationClass::accxy_stab_rep(float speed, float maxA, float timeL) {
	float t = speed / maxA + timeL;
	return 0.5*maxA*t*t;
}

void StabilizationClass::setDefaultMaxSpeeds(){//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	max_speed_xy = MAX_HOR_SPEED;
	max_speedZ_P = MAX_VER_SPEED_PLUS;
	max_speedZ_M = MAX_VER_SPEED_MINUS;
}


void StabilizationClass::max_speed_limiter(float &x, float &y) {
	const float speed2 = (x*x + y * y);
	const float maxSpeed2 = max_speed_xy * max_speed_xy;
	if (speed2 > maxSpeed2) {
		const float k = (float)sqrt(maxSpeed2 / speed2);
		x *= k;
		y *= k;
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


void StabilizationClass::fromLoc2Pos(long lat, long lon, float &x, float &y) {
	GPS.loc.fromLoc2Pos(lat, lon, x, y);
	Mpu.getXYRelative2Zero(x, y);
}
void StabilizationClass::setNeedLoc(long lat, long lon, float &x, float &y) {
	fromLoc2Pos(lat, lon, x, y);
	setNeedPos(x, y);
	
}


void StabilizationClass::add2NeedPos(float speedX, float speedY, float dt) {
	static bool flagzx = false;
	static bool flagzy = false;
	if (speedX == 0) {
		if (flagzx == false) {
			flagzx = true;
			float x, y;
			needXR = needXV = Mpu.get_Est_X();
		}
	}
	else {
		flagzx = false;
		float distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needXR += speedX * dt;
		needXV = needXR + distX;
	}
	if (speedY == 0) {
		if (flagzy == false) {
			flagzy = true;
			float x, y; 
			needYV = needYR = Mpu.get_Est_Y();
		}
	}
	else {
		flagzy = false;
		float distX = speedX, distY = speedY;
		speed2dist(distX, distY);
		needYR += speedY * dt;
		needYV = needYR + distY;
	}
}
float StabilizationClass::get_dist2goal(){
	float dx= Mpu.get_Est_X() - needXV;
	float dy = Mpu.get_Est_Y() - needYV;
	return sqrt(dx*dx + dy * dy);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//float old_gps_bearing = 0, cos_bear = 1,  sin_bear = 0;
void StabilizationClass::XY(float &pitch, float&roll){//dont work 
		float need_speedX, need_speedY;
		static float naccXF = 0, naccYF = 0;
		float tx, ty;
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
	//	const float accX_C =  ((d_speedX * speed_2_acc_XY) - Mpu.w_accX)*acc_2_angle;
	//	const float accY_C = ((d_speedY * speed_2_acc_XY) - Mpu.w_accY)*acc_2_angle;

		const float w_pitch = -(pids[SPEED_X_SPEED].get_pid(d_speedX, Mpu.dt));
		const float w_roll = pids[SPEED_Y_SPEED].get_pid(d_speedY, Mpu.dt);

		

		//----------------------------------------------------------------преобр. в относительную систему координат
		pitch = Mpu.cosYaw*w_pitch - Mpu.sinYaw*w_roll;
		roll = Mpu.cosYaw*w_roll + Mpu.sinYaw*w_pitch;

		//Debug.load(0, pitch, roll);
		//Debug.dump();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float StabilizationClass::Z(){


	//-------------stab

	const float need_speedZ = getSpeed_Z(Autopilot.fly_at_altitude() - Mpu.get_Est_Alt());

	mc_z += (need_speedZ - Mpu.get_Est_SpeedZ() - mc_z)*Z_FILTER;
	//const float accZ_C = ((mc_z * speed_2_acc_Z) - Mpu.faccZ)*acc_2_power;



	float fZ = HOVER_THROTHLE +  pids[SPEED_Z_PID].get_pid(mc_z, Mpu.dt)*Balance.powerK();
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
	int error = 1;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK) {
		error = 0;
		float t;
		error += Settings._set(ar[i++], alt2speedZ);
		t = pids[SPEED_Z_PID].kP();
		if ((error += Settings._set(ar[i++], t)) == 0)
			pids[SPEED_Z_PID].kP(t);
		t = pids[SPEED_Z_PID].kI();
		if ((error += Settings._set(ar[i++], t)) == 0)
			pids[SPEED_Z_PID].kI(t);
		error += Settings._set(ar[i++], max_speedZ_P);
		error += Settings._set(ar[i++], max_speedZ_M);
		error += Settings._set(ar[i++], Z_FILTER);
	}
	if (error == 0) {
		cout << "Stabilization Z set:\n";
		for (uint8_t ii = 0; ii < i; ii++) {
			cout << ar[ii] << ",";
		}
		cout << endl;
	}
	else
		cout << "ERROR to big or small. P=" << error;
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

	int error = 1;
	uint8_t i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){

		error = 0;
		float t;
		

		error += Settings._set(ar[i++], dist2speed_XY);
		t = pids[SPEED_X_SPEED].kP();
		if ((error += Settings._set(ar[i++], t))==0)
			set_acc_xy_speed_kp(t);
		t = pids[SPEED_X_SPEED].kI();
		if ((error += Settings._set(ar[i++], t))==0)
			set_acc_xy_speed_kI(t);
		error += Settings._set(ar[i++], max_speed_xy);
		error += Settings._set(ar[i++], XY_FILTER);

	}
	if (error == 0) {
		cout << "Stabilization XY set : \n";

		for (uint8_t ii = 0; ii < i; ii++) {
			cout << ar[ii] << ",";
		}
		cout << endl;
	}
	else
		cout << "ERROR to big or small. P=" << error;

	
}












StabilizationClass Stabilization;
