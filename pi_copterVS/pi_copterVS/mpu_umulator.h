


#pragma once
#include "define.h"
#include <math.h>

# define M_E		2.7182818284590452354	/* e */
# define M_LOG2E	1.4426950408889634074	/* log_2 e */
# define M_LOG10E	0.43429448190325182765	/* log_10 e */
# define M_LN2		0.69314718055994530942	/* log_e 2 */
# define M_LN10		2.30258509299404568402	/* log_e 10 */
# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */
# define M_PI_4		0.78539816339744830962	/* pi/4 */
# define M_1_PI		0.31830988618379067154	/* 1/pi */
# define M_2_PI		0.63661977236758134308	/* 2/pi */
# define M_2_SQRTPI	1.12837916709551257390	/* 2/sqrt(pi) */
# define M_SQRT2	1.41421356237309504880	/* sqrt(2) */
# define M_SQRT1_2	0.70710678118654752440	/* 1/sqrt(2) */

enum { PITCH,ROLL,YAW };


class EmuClass
{
private:


	double fm[4] = { 0,0,0,0 };

	double ang[3] = { 0,0,0 };
	double gyro[3] = { 0,0,0 };
	double pos[3] = { 0,0,0 };

	double speed[3] = { 0,0,0 };
	double acc[3] = { 0,0,0 };
	double resistenceF[3] = { 0,0,0 };
	double gyro_res[3] = { 0,0,0 };

	float wind[3];
	float f[4][3];
public:
	void battery(float [], float &voltage);
	void init(float wx, float wy, float wz, float yaw=0,float pitch=0,float roll=0);
	float get_pitch();
	float get_roll();
	float get_yaw();
	float get_heading();
	float get_gyroYaw();
	float get_gyroPitch();
	float get_gyroRoll();
	float get_accX();
	float get_accZ();
	float get_accY();

	float get_alt();
	float get_speedZ();
	float get_x();
	float get_y();

	void update(const float fm[4], double dt);
	void init(float wind[]);
	EmuClass();
	~EmuClass();
};

extern EmuClass Emu;
