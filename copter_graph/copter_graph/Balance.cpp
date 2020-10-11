#include "stdafx.h"
#include "Balance.h"
#include "PID.h";
#include "Mpu.h"

float wind_i_X=0, wind_i_Y=0, wind_i_max=15, windRC=0.1f;

#define _PITCH 0
#define _ROLL 1

void to_max_ang(const float ang, float& angX, float& angY) {
	float k = sqrt(angX * angX + angY * angY);
	if (k > ang) {
		k = ang / k;
		angX *= k;
		angY *= k;
	}
}
void Balance::calculate_wind_i(float world_ang[]) {

	if (

		Mpu.get_est_LF_hor_acc() < 0.5f &&

		abs(Mpu.get_est_LF_ver_acc()) < 0.5f)
	{
		
		//wind_i_X += (world_ang[_PITCH] - wind_i_X) * 0.0001;
	//	wind_i_Y += (world_ang[_ROLL] - wind_i_Y) *  0.0001;
	//	to_max_ang(wind_i_max, wind_i_X, wind_i_Y);

	}

	if (thr >= 0.35) {
		if (
			Mpu.get_est_LF_hor_speed() < 0.6f &&
			Mpu.get_est_LF_hor_acc() < 0.5f &&
			abs(Mpu.get_est_LF_ver_speed()) < 0.6f &&
			abs(Mpu.get_est_LF_ver_acc()) < 0.5f)
		{

			const float owx = wind_i_X;
			const float owy = wind_i_Y;
			wind_i_X += (world_ang[_PITCH] - wind_i_X) * windRC;
			wind_i_Y += (world_ang[_ROLL] - wind_i_Y) * windRC;
			to_max_ang(wind_i_max, wind_i_X, wind_i_Y);

			//world_ang[_PITCH] -= (wind_i_X - owx);
			//world_ang[_ROLL] -= (wind_i_Y - owy);
			//Debug.dump(wind_i_X, wind_i_Y, 0, 0);
			//const float wind_i = sqrt(wind_i_X * wind_i_X + wind_i_Y * wind_i_Y);
			//pid_hor.set_kI_max(Balance.get_max_angle() - wind_i);
			//pid_hor.hi_2_error_max_diff(Balance.get_max_angle() - wind_i);


		}
	}
	else {
		wind_i_X = wind_i_Y = 0;
	}
}



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
	tx = Mpu.estX - Mpu.start_pos[mEX];
		need_speedX = (tx - 0);
		ty = Mpu.estY-Mpu. start_pos[mEY];
		need_speedY = (ty - 0);

	//}//вичислять нужное ускорение по форумуле a=v*v/(2s)
	dist2speed(need_speedX, need_speedY);
	d_speedX += ((need_speedX +Mpu.est_speedX) - d_speedX) * 1;
	d_speedY += ((need_speedY +Mpu.est_speedY) - d_speedY) * 1;


	const float w_pitch = -(pidx.get_pid(d_speedX, Mpu.dt));
	const float w_roll = pidy.get_pid(d_speedY, Mpu.dt);

	

	//----------------------------------------------------------------преобр. в относительную систему координат
	pitch = (float)(Mpu.cosYaw * w_pitch - Mpu.sinYaw * w_roll);
	roll = (float)(Mpu.cosYaw * w_roll + Mpu.sinYaw * w_pitch);


	//pitch = w_pitch;
	//roll = w_roll;


	//Debug.load(0, pitch, roll);
	//Debug.dump();
}

















void Balance::parser(byte buf[], int i, uint32_t cb, bool filtr , bool rot) {
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

	if ((cb & MOTORS_ON) && (cb & XY_STAB)) {
		float t[2];
		t[0] = ap_pitch;
		t[1] = ap_roll;

		calculate_wind_i(t);
		if (filtr) {
			ap_pitch = wind_i_X;
			ap_roll = wind_i_Y;
		}
	}


														  
	





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


