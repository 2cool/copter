#include "stdafx.h"
#include "Balance.h"
#include "PID.h";
int Balance::decode(char buffer[], int &i, bool rotate) {
	i += 9;
	f[bcPITCH]=*(float*)(&buffer[i]);
	i += 4;
	f[bcROLL] = *(float*)(&buffer[i]);
	i+= 4;
	f[bTHROTHLE] = *(float*)(&buffer[i]);
	i += 4;
	int n=*(int*)(&buffer[i]);
	i += 4;
	i += 2;
	f[bF0] = *(float*)(&buffer[i]);
	i += 4;
	f[bF1] = *(float*)(&buffer[i]);
	i += 4;
	f[bF2] = *(float*)(&buffer[i]);
	i += 4;
	f[bF3] = *(float*)(&buffer[i]);
	i += 4;

	return 0;
}


float dist2speed_XY = 0.2;


AP_PID pidx, pidy;
void set_acc_xy_speed_kp(const float f) { pidx.kP(f);	pidy.kP(f); }
void set_acc_xy_speed_kI(const float f) { pidx.kI(f);	pidy.kI(f); }
void set_acc_xy_speed_kD(const float f) { pidx.kD(f, 3);	pidy.kD(f, 3); }
void set_acc_xy_speed_imax(const float f) { pidx.imax(-f, f);	pidy.imax(-f, f); }

void Balance::init()
{
	for (int i = 0; i < bALL; i++) {
		_max[i] = -1000000;
		_min[i] =  1000000;
	}

	// 0.2336, 7.09687, 2.5, 10, 1, 1, 1, 1, 1, 1, 1,

	dist2speed_XY = 0.2336;//0.5
	set_acc_xy_speed_kp(7.09687);
	set_acc_xy_speed_kI(2.5);
	set_acc_xy_speed_imax(35);
	










	


}
#include "Mpu.h"




uint32_t control_bits_ = 0;

float max_speed_xy = 10;


float d_speedX = 0, d_speedY = 0;

void max_speed_limiter(float& x, float& y) {
	const double speed2 = (x * x + y * y);
	const double maxSpeed2 = max_speed_xy * max_speed_xy;
	if (speed2 > maxSpeed2) {
		const float k = (float)sqrt(maxSpeed2 / speed2);
		x *= k;
		y *= k;
	}
}
void dist2speed(float& x, float& y) {
	x = dist2speed_XY * x;
	y = dist2speed_XY * y;
	max_speed_limiter(x, y);

}



void XY(float& pitch, float& roll) {//dont work 
	float need_speedX, need_speedY;
	float tx, ty;
	//if ((control_bits_& XY_STAB & 1) == 9){
	tx = mpu.estX - mpu.start_pos[mEX];
		need_speedX = (tx - 0);
		ty = mpu.estY-mpu. start_pos[mEY];
		need_speedY = (ty - 0);

	//}//вичислять нужное ускорение по форумуле a=v*v/(2s)
	dist2speed(need_speedX, need_speedY);
	d_speedX += ((need_speedX +mpu.est_speedX) - d_speedX) * 1;
	d_speedY += ((need_speedY +mpu.est_speedY) - d_speedY) * 1;


	const float w_pitch = -(pidx.get_pid(d_speedX, mpu.dt));
	const float w_roll = pidy.get_pid(d_speedY, mpu.dt);

	

	//----------------------------------------------------------------преобр. в относительную систему координат
	pitch = (float)(mpu.cosYaw * w_pitch - mpu.sinYaw * w_roll);
	roll = (float)(mpu.cosYaw * w_roll + mpu.sinYaw * w_pitch);


	//pitch = w_pitch;
	//roll = w_roll;


	//Debug.load(0, pitch, roll);
	//Debug.dump();
}

















void Balance::parser(byte buf[], int i, uint32_t cb) {
	control_bits_ = cb;

	int bank_counter = *(uint32_t*)&buf[i];// load_int32(buf, i);
	i += 4;
	float *fb = (float*)&buf[i];
	f0 = fb[0];
	f1 = fb[1];
	f2 = fb[2];
	f3 = fb[3];
	thr += ((f0 + f1 + f2 + f3)*0.25-thr)*1;

	i += 16;
	ap_roll = 1.0 / 16 * (double)(*(int16_t*)&buf[i]);
	i += 2;
	ap_pitch = 1.0 / 16 * (double)(*(int16_t*)&buf[i]); ;
														  
	i += 2;
	ap_yaw = 1.0 / 16 * (double)(*(int16_t*)&buf[i]); ;





	if (control_bits_ & 1) {
float pitch, roll;
	XY(pitch, roll);
	//ap_pitch = ap_roll;
	//ap_roll = roll;

	}
	else {
		pidx.reset_I();
		pidy.reset_I();
		//zero_x = mpu.estX;
		//zero_y = mpu.estY;
	}
	//calculations
	





}





Balance bal;


