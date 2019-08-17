#include "mpu.h"
#include "define.h"
#include "Settings.h"
#include "Hmc.h"
#include "Autopilot.h"
#include "Telemetry.h"
#include "Balance.h"
#include "debug.h"
#include "Stabilization.h"
#include "GPS.h"
#include "Log.h"
#include "MadgwickAHRS.h"

/*

пользоватся только гироскопом если акселерометр показивает меньший угол. (но временно) и акселерометр и гироскоп должни бить отфильтровані
при снижении резком. надо тоже обдумть  логику.


*/





//#include "Mem.h"


#define delay_ms(a)    usleep(a*1000)


double  mpu_time_ ;



#define RESTRICT_PITCH // Comment out to restrict roll to ±M_2PIdeg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//2g
//	0.00006103515625
//4g

//3G



#ifdef USE_KALMAN
const int mn = 3; // Number of states
const int m = 1; // Number of measurements
Eigen::MatrixXd A(mn, mn); // System dynamics matrix
Eigen::MatrixXd H(m, mn); // Output matrix
Eigen::MatrixXd Q(mn, mn); // Process noise covariance
Eigen::MatrixXd R(m, m); // Measurement noise covariance
Eigen::MatrixXd P(mn, mn); // Estimate error covariance
KalmanFilter* kf;
#endif



static const float f_constrain(const float v, const float min, const float max){
	return constrain(v, min, max);
}
#define DIM 3
//WiFiClass wi_fi;

float MpuClass::get_yaw() { return yaw; }
//-----------------------------------------------------
void  MpuClass::initYaw(const float angle){
	yaw = angle;
}
//-----------------------------------------------------
float MpuClass::get_pitch() { return pitch; }
float MpuClass::get_roll() { return roll; }

//-----------------------------------------------------------

void MpuClass::set_XYZ_to_Zero() {
	XatZero = estX;
	YatZero = estY;
	altitude_at_zero = est_alt_;
}


//-----------------------------------------------------

void MpuClass::log() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::MPU_SENS);

		Log.loaduint64t(timed *1000);//new
		Log.loadMem((uint8_t*)g, 6, false);
		Log.loadMem((uint8_t*)a, 6, false);

		Log.loadFloat(pitch);
		Log.loadFloat(roll);
		Log.loadFloat(yaw);
		Log.loadFloat(gyroPitch);
		Log.loadFloat(gyroRoll);
		Log.loadFloat(gyroYaw);
		Log.loadFloat(accX);
		Log.loadFloat(accY);
		Log.loadFloat(accZ);

		Log.loadFloat(est_alt_);
		Log.loadFloat(est_speedZ);
		Log.loadFloat(get_Est_X());
		Log.loadFloat(est_speedX);
		Log.loadFloat(get_Est_Y());
		Log.loadFloat(est_speedY);

		Log.block_end();
	}
}
void MpuClass::log_emu() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::MPU_EMU);

		Log.loadByte((uint8_t)(dt * 1000));
		Log.loadFloat(pitch);
		Log.loadFloat(roll);
		Log.loadFloat(yaw);
		Log.loadFloat(gyroPitch);
		Log.loadFloat(gyroRoll);
		Log.loadFloat(gyroYaw);
		Log.loadFloat(accX);
		Log.loadFloat(accY);
		Log.loadFloat(accZ);

		Log.block_end();
	}


}
//-----------------------------------------------------
//-----------------------------------------------------
void MpuClass::init()
{
	yaw_correction_angle = 19 * GRAD2RAD;

	tiltPower_CF = 0.05;
	altitude_at_zero = XatZero = YatZero = 0;
	_0007=0.007;
	acc_callibr_timed = 0;
	w_accX = w_accY = yaw_off = 0;
	fx = fy = fz = sinPitch = sinRoll = 0;
	yaw_offset = yaw = pitch = roll = gyroPitch = gyroRoll = gyroYaw = accX = accY = accZ = 0;

	max_g_cnt = 0;
	cosYaw = 1;
	sinYaw = 0;
	temp_deb = 6;
	
	
	tiltPower = cosPitch = cosRoll = 1;

	//COMP_FILTR = 0;// 0.003;
	vibration = 0;

	q.w = 1; q.x = q.y = q.z = 0;
	oldmpuTimed = mpu_time_ = 0.000001*micros();




#ifdef USE_KALMAN

	// Discrete LTI projectile motion, measuring position only
	A <<
		1, 0.005, 0,
		0, 1, 0.005,
		0, 0, 1;

	H << 1, 0, 0;

	// Reasonable covariance matrices
#define newQ 0.001
#define newR 0.2

	Q <<
		newQ, newQ, .0,
		newQ, newQ, .0,
		.0, .0, .0;

	R << newR;

	P <<
		.1, .1, .1,
		.1, 10000, 10,
		.1, 10, 100;


	VectorXd B(mn);
	B << 0, 0, 0;
	// Construct the filter
	kf = new KalmanFilter(A, B, H, Q, R, P);

	// Construct the filter

	Eigen::VectorXd x0(mn);
	x0 << 0, 0, 0;
	kf->init(x0);


#endif








	cout << "Initializing MPU6050\n";

#ifndef FLY_EMULATOR




	accelgyro.initialize(MPU6050_GYRO_FS_1000, MPU6050_ACCEL_FS_2, MPU6050_DLPF_BW_20);
	//ms_open();
	sleep(1);
	accelgyro.initialize(MPU6050_GYRO_FS_1000, MPU6050_ACCEL_FS_2, MPU6050_DLPF_BW_20);
	writeWord(104, MPU6050_RA_XA_OFFS_H, -535);//-5525);
	writeWord(104, MPU6050_RA_YA_OFFS_H, 219);// -1349);
	writeWord(104, MPU6050_RA_ZA_OFFS_H, 1214);// 1291);
	writeWord(104, MPU6050_RA_XG_OFFS_USRH, 165);// -43);
	writeWord(104, MPU6050_RA_YG_OFFS_USRH, -39);// 36);
	writeWord(104, MPU6050_RA_ZG_OFFS_USRH, 16);// -49);
		
	

	
	


#ifdef GYRO_CALIBR
	gyro_calibratioan = false;
#else
	gyro_calibratioan = true;
#endif

#ifdef DEBUG_MODE
		//for (int i = 0; i < 6; i++)
		//	Out.println(offset_[i]);
#endif
#ifndef WORK_WITH_WIFI
		//calibrated = false;
#endif
	//}
//	else {
	//	cout << "MPU NOT CALIBRATED !!!\n");
	//}




#else
	

#endif

}
//-----------------------------------------------------
string MpuClass::get_set(){
	
	
	ostringstream convert;
	convert << "0,0";
		
	
	string ret = convert.str();
	return string(ret);
	
}
//-----------------------------------------------------
void MpuClass::set(const float  *ar){

	int i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		int error = 0;
		/*
		float t;

		t = DRAG_K;
		if (error += Settings._set(ar[i++], t) == 0)
			DRAG_K = t;
		t = _0007;
		if (error += Settings._set(ar[i++], t) == 0)
			_0007 = t;
		t = tiltPower_CF;
		if (error += Settings._set(ar[i++], t) == 0)
			tiltPower_CF = t;
		cout << "mpu set:\n";
		//int ii;
		if (error == 0){
			//for (ii = 0; ii < i; ii++){
			//	Out.cout << ar[ii]); Out.cout << ",");
			//}
			//Out.println(ar[ii]);
			cout << "OK\n";
		}
		else{
			cout << "ERROR to big or small. P=" << error << endl;
		}
		*/
	}
	else{
		cout << "ERROR\n";
	}
}
//-----------------------------------------------------
int16_t MpuClass::getGX(){
	int16_t x, y, z;
	accelgyro.getAcceleration(&x, &y, &z);
	return x;
}
//-----------------------------------------------------
const float n003 = 0.030517578f;
//const float n006 =  0.061035156f;
//4g
//const float n122 = 1.220740379e-4;
const float n305 = 3.0518509475e-5;

//2g
//const float n604 = 0.00006103515625f;



const float to_98g = 0.0005981445312f;

#ifdef FLY_EMULATOR





float real_pitch = 0, real_roll = 0;

//-----------------------------------------------------

/*
void MpuClass::calc_corrected_ang(){

	double raccX = cosYaw*GPS.loc.accX + sinYaw*GPS.loc.accY;
	double raccY = cosYaw*GPS.loc.accY - sinYaw*GPS.loc.accX;






}
*/
///////////////////////////////////////////////////////////////////

bool MpuClass::loop(){

	timed = (0.000001*(double)micros());
	double ___dt = (float)(timed - oldmpuTimed);// *div;
	if (___dt < 0.01)
		return false;
	Hmc.loop();
	MS5611.loop();
	GPS.loop();
	dt = 0.01;
	rdt = 1.0 / dt;
	mpu_dt = dt;
	oldmpuTimed = timed;
	if (dt > 0.02)
		dt = 0.01;


	pitch=Emu.get_pitch();
	roll = Emu.get_roll();



	gyroPitch = Emu.get_gyroPitch();
	gyroRoll = Emu.get_gyroRoll();
	gyroYaw = Emu.get_gyroYaw();
	


	

	float head = Emu.get_heading();

	float g_yaw = Emu.get_yaw();

	yaw_offset += (wrap_PI(g_yaw - head) - yaw_offset)*0.0031f;

	yaw = wrap_PI(g_yaw - yaw_offset+ yaw_correction_angle );


	sin_cos(pitch, sinPitch, cosPitch);
	sin_cos(roll, sinRoll, cosRoll);


	tiltPower = cosPitch*cosRoll;
	cosYaw = cos(yaw);
	sinYaw = sin(yaw);


	float WaccX = Emu.get_accX();
	float WaccY = Emu.get_accY();
	accZ = Emu.get_accZ();

	//faccZ += (accZ - faccZ)*ACC_Z_CF;

	accX = (cosYaw * WaccX + sinYaw * WaccY); //relative to copter xy
	accY = (cosYaw * WaccY - sinYaw * WaccX);

	test_Est_Alt();
	test_Est_XY();

	yaw *= RAD2GRAD;
	pitch *= RAD2GRAD;
	roll *= RAD2GRAD;

	
	//r_pitch = RAD2GRAD*pitch;
	//r_roll = RAD2GRAD*roll;

	delay(1);
	gyro_calibratioan = true;

	log_emu();

	shmPTR->pitch = pitch;
	shmPTR->roll = roll;
	shmPTR->yaw = yaw;

	return true;
}

#else


/////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t GetGravity(VectorFloat *v, Quaternion_ *q) {
	v->x = 2 * (q->x*q->z - q->w*q->y);
	v->y = 2 * (q->w*q->x + q->y*q->z);
	v->z = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
	return 0;
}


#define ROLL_COMPENSATION_IN_YAW_ROTTATION 0.02
#define PITCH_COMPENSATION_IN_YAW_ROTTATION 0.025

float agpitch = 0, agroll = 0, agyaw = 0;
int32_t cal_g_pitch = 0, cal_g_roll = 0, cal_g_yaw = 0,  cal_g_cnt = 0;
uint64_t maxG_firs_time = 0;


bool compas_flip = false;
#define _2PI 6.283185307179586476925286766559
bool pitch_flag;

static void toEulerianAngle(const Quaternion_& q, float& roll, float& pitch, float& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);

	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	yaw = atan2(siny, cosy);
}

void MpuClass::test_vibration( float x,  float y,  float z){
	static float lx = 0, ly = 0, lz = 0;
	lx += (x - lx)*0.1;
	ly += (y - ly)*0.1;
	lz += (z - lz)*0.1;
	x -= lx;
	y -= ly;
	z -= lz;
	const float vibr = sqrt(x * x + y * y + z * z);
	vibration += (vibr - vibration)*0.01;
	
}


//------------------------------------------------------------------------------------
void MpuClass::gyro_calibr() {
	if (Autopilot.motors_is_on() == false) {

		if (timed < 30 || acc_callibr_timed > timed) {

			AHRS.setBeta(0.1);
			if (cal_g_cnt == 0) {
				cal_g_pitch = cal_g_roll = cal_g_yaw = 0;
			}
			cal_g_pitch += g[1];
			cal_g_roll += g[0];
			cal_g_yaw += g[2];
			cal_g_cnt++;
			agpitch = n003 * ((double)cal_g_pitch / (double)cal_g_cnt);
			agroll = n003 * ((double)cal_g_roll / (double)cal_g_cnt);
			agyaw = n003 * ((double)cal_g_yaw / (double)cal_g_cnt);
		}
		else {
			AHRS.setBeta(0.01);
			cal_g_cnt = 0;
		}
	}
}
double gravity = 1;





bool MpuClass::loop() {//-------------------------------------------------L O O P-------------------------------------------------------------
	
	bool ret = true;
	timed = 0.000001*(double)micros();
	mpu_dt = timed - mpu_time_;
	mpu_dt = constrain(mpu_dt,0.005, 0.03);
	mpu_time_ = timed;
	double old_tied = timed;

	double _dt = (timed - oldmpuTimed);
	if (ret=(_dt >= 0.01)) {
		Hmc.loop();
		oldmpuTimed = timed;
		dt = constrain(_dt, 0.01, 0.03);
		rdt = 1.0 / dt;

		static uint cnt2l = 0;
		if (dt > 0.03) {
			if (cnt2l++) {
				//cout << "MPU DT too long " << endl;// << dt << ":" << dt << ":" << timed << endl;
				//cout << endl << ms5611_timed - old_tied << " " << hmc_timed - old_tied << " " << gps_timed - old_tied <<
					//" " << telem_timed - old_tied << " " << com_timed - old_tied << " " << autopilot_timed - old_tied << " " << mpu_timed - old_tied << endl;
				mega_i2c.beep_code(B_MPU_TOO_LONG);
				Telemetry.addMessage(e_MPU_TOO_LONG);
			}
		}
	}
	
	if (accelgyro.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]) == -1) {
		Telemetry.addMessage(e_MPU_RW_ERROR);
		mega_i2c.beep_code(B_I2C_ERR);
		return ret;
	}

	gyroPitch =  n003 * (float)g[1] - agpitch;
	gyroRoll =  n003 * (float)g[0] - agroll;
	gyroYaw =  n003 * (float)g[2] - agyaw;
	float ax = n305 * 2 * (float)a[0];
	float ay = n305 * 2 * (float)a[1];
	float az = n305 * 2 * (float)a[2];
	//if (az > 3)
	//	cout<<"AZM\n" ;//??????????????????????????????????????

 	gravity+=(sqrt(ax * ax + ay * ay + az * az)-gravity)*0.05;
	if (gravity > 1.1 || gravity < 0.9) {
		static int cntttttsss = 0;
		//cout << cntttttsss++ << " gravity error\n";
	}

	AHRS.MadgwickAHRSupdate(q, GRAD2RAD * gyroRoll, GRAD2RAD * gyroPitch, GRAD2RAD * gyroYaw, ax, ay, az,  Hmc.fmx, Hmc.fmy, Hmc.fmz, mpu_dt);
	//AHRS.MadgwickAHRSupdate(q, GRAD2RAD * gyroRoll, GRAD2RAD * gyroPitch, GRAD2RAD * gyroYaw, 0, 0, 10, Hmc.fmx, Hmc.fmy, -Hmc.fmz, mpu_dt);
	gyro_calibr();

	toEulerianAngle(q, roll, pitch, yaw);

	gyroPitch = -gyroPitch;
	gyroYaw = -gyroYaw;
	pitch = -pitch;
	yaw = -yaw;
	yaw += yaw_correction_angle;
	yaw=wrap_PI(yaw);

	sin_cos(yaw, sinYaw, cosYaw);
	sin_cos(pitch, sinPitch, cosPitch);
	sin_cos(roll, sinRoll, cosRoll);

	tiltPower += (constrain(cosPitch*cosRoll, 0.5f, 1) - tiltPower)*tiltPower_CF;
	
	accZ = az*cosPitch + sinPitch*ax;
	accZ = 9.8f*(accZ*cosRoll + sinRoll*ay - 1);

	accX = 9.8f*(ax*cosPitch - az*sinPitch);
	accY = 9.8f*(-ay*cosRoll + az*sinRoll);

	test_vibration(accX, accY, accZ);

	test_Est_Alt();
	test_Est_XY();

	shmPTR->pitch = pitch *= RAD2GRAD;
	shmPTR->roll = roll *= RAD2GRAD;
	shmPTR->yaw = yaw*=RAD2GRAD;
	log();
	return ret;
}


#endif


void MpuClass::setDLPFMode_(uint8_t bandwidth){
	gLPF = bandwidth;
	accelgyro.setDLPFMode(bandwidth);
}




void MpuClass::new_calibration(const bool onlyGyro){

	acc_callibr_timed = timed+10;
	gyro_calibratioan = true;
}








//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
если фаза между distX and distY имеет угол.
то єто угол между реальним севером и тем что показивает компас.

если distX and distY в фазе но колебаются то увеличить SPEED_X_SPEED kp, SPEED_X_SPEED kp

*/
float Z_CF_DIST = 0.015;//CF filter
float Z_CF_SPEED = 0.0025;//CF filter //при 0.005 На ошибку в ACC на 0.1 ошибка в исоте на метр.
float AltErrorI=0;
void MpuClass::test_Est_Alt() {


	
	float alt = MS5611.alt();
	   	 	


#ifdef USE_KALMAN

	float static old_bar_alt = 0,old_accZ=0;
	kf->A(3) = kf->A(7)= mpu_dt;
	kf->B[2] = accZ - old_accZ;
	old_accZ = accZ;

	if (alt != old_bar_alt) { 
		old_bar_alt = alt;
		Eigen::VectorXd y(m);
		y << old_bar_alt;
		kf->update(y);

	}
	else {
		kf->update();
	}
	est_speedZ = kf->state()(1);
	est_alt_ = kf->state()(0);
	
	Debug.dump(est_alt_, est_speedZ, 0, 0);

#else
	if (timed<8) {
		est_alt_ = alt;
		est_speedZ = 0;
		return;
	}
	
	float acc = accZ + AltErrorI;
	
	est_alt_ += mpu_dt*(est_speedZ + acc*mpu_dt*0.5f);
	est_speedZ += acc*mpu_dt;

	//est_speedZ += acc * mpu_dt;
	//est_alt_ += est_speedZ * mpu_dt;

	est_alt_ += (alt - est_alt_)*Z_CF_DIST;
	est_speedZ += (MS5611.speed - est_speedZ)*Z_CF_SPEED;

	AltErrorI += (alt - est_alt_ - AltErrorI)*0.0001;
	AltErrorI = constrain(AltErrorI, -1, 1);


	//Debug.load(0, AltErrorI, alt - est_alt_,est_alt_);
	//Debug.dump(true);

//	AltError = alt - est_alt_;
//	AltErrorI += AltError;
//	AltErrorI = constrain(AltErrorI, -10000.0f, 10000.0f);

#endif

#ifdef FLY_EMULATOR
	if (alt <= 0) {
		est_alt_ = 0;
		est_speedZ = 0;
	}

#endif

	//Debug.load(0, est_speedZ, est_alt_);
	//Debug.dump();
	
	//static float old1 = 0, old2 = 0;
	//Debug.load(0, est_alt_-old1, alt-old2);
	//old1 = est_alt_;
	//old2 = alt;
	//Debug.dump();
	
	
}

void MpuClass::rotateCW(float &x, float &y) {
	const float _x = (cosYaw * x - sinYaw * y); //relative to copter xy
	const float _y = (cosYaw * y + sinYaw * x);
	x = _x;
	y = _y;
}
void MpuClass::rotateCCW(float &x, float &y) {
	const float _x = (cosYaw * x + sinYaw * y);
	const float _y = (cosYaw * y - sinYaw * x);
	x = _x;
	y = _y;
}


float XY_KF_DIST = 0.05f;
float XY_KF_SPEED = 0.05f;

float est_XError = 0, est_XErrorI = 0;
float est_YError = 0, est_YErrorI = 0;
#define ACC_Cr 10000.0f
void MpuClass::test_Est_XY() {
	//accX = 0.9;
	//accY = 0.5;
	if (GPS.loc.dX == 0 && GPS.loc.dY == 0) {
		est_XError = est_XErrorI = est_YError = est_YErrorI = 0;
		estX = estY = est_speedX = est_speedY = 0;
	}
	est_XError = GPS.loc.dX - estX;
	est_YError = GPS.loc.dY - estY;
	float e_ex = (-cosYaw * est_XError - sinYaw * est_YError); //relative to copter xy
	float e_ey = (-cosYaw * est_YError + sinYaw * est_XError);
	
	/*

	est_XErrorI += e_ex;
	est_XErrorI = constrain(est_XErrorI, -ACC_Cr, ACC_Cr);
	est_YErrorI += e_ey;
	est_YErrorI = constrain(est_YErrorI, -ACC_Cr, ACC_Cr);
	float c_accX = accX + est_XErrorI / ACC_Cr;
	float c_accY = accY + est_YErrorI / ACC_Cr;
	*/
	est_XErrorI += (e_ex - est_XErrorI)*0.0001;
	//est_XErrorI = constrain(est_XErrorI, -1, 1);
	float c_accX = accX + est_XErrorI;

	est_YErrorI += (e_ey - est_YErrorI)*0.0001;
	//est_YErrorI = constrain(est_YErrorI, -1, 1);
	float c_accY = accY + est_YErrorI;



	
	w_accX = (-cosYaw*c_accX + sinYaw*c_accY); //relative to world
	w_accY = (-cosYaw*c_accY - sinYaw*c_accX);

	//--------------------------------------------------------estimate
	estX += mpu_dt*(est_speedX + w_accX * mpu_dt*0.5f);
	est_speedX += (w_accX*mpu_dt);
	estY += mpu_dt*(est_speedY + w_accY * mpu_dt*0.5f);
	est_speedY += (w_accY*mpu_dt);
	// -------------------------------------------------------corection
	estX += (GPS.loc.dX - estX)*XY_KF_DIST;
	est_speedX += (GPS.loc.speedX - est_speedX)*XY_KF_SPEED;
	//--------------------------------------------------------
	estY += (GPS.loc.dY - estY)*XY_KF_DIST;
	est_speedY += (GPS.loc.speedY - est_speedY)*XY_KF_SPEED;
	
	//Debug.load(0, Mpu.w_accX, Mpu.w_accY);
	//Debug.dump();
}

/*
void MpuClass::set_cos_sin_dir() {

		double angle = atan2(est_speedY, est_speedX);

		dir_angle_GRAD = angle*RAD2GRAD;
		//ErrorLog.println(angle*RAD2GRAD);
		cosDirection = fabs(cos(angle));
		sinDirection = fabs(sin(angle));

}
*/



MpuClass Mpu;



