
//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>



#define betaDef		0.9f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
float beta = betaDef;								// 2 * proportional gain (Kp)
void MadgwickAHRS::setBeta(float b) { beta = b; }
void MadgwickAHRS::MadgwickAHRSupdate(Quaternion_ &q,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(q,gx, gy, gz, ax, ay, az, dt);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
	qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
	qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
	qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q.w * mx;
		_2q0my = 2.0f * q.w * my;
		_2q0mz = 2.0f * q.w * mz;
		_2q1mx = 2.0f * q.x * mx;
		_2q0 = 2.0f * q.w;
		_2q1 = 2.0f * q.x;
		_2q2 = 2.0f * q.y;
		_2q3 = 2.0f * q.z;
		_2q0q2 = 2.0f * q.w * q.y;
		_2q2q3 = 2.0f * q.y * q.z;
		q0q0 = q.w * q.w;
		q0q1 = q.w * q.x;
		q0q2 = q.w * q.y;
		q0q3 = q.w * q.z;
		q1q1 = q.x * q.x;
		q1q2 = q.x * q.y;
		q1q3 = q.x * q.z;
		q2q2 = q.y * q.y;
		q2q3 = q.y * q.z;
		q3q3 = q.z * q.z;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q.z + _2q0mz * q.y + mx * q1q1 + _2q1 * my * q.y + _2q1 * mz * q.z - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q.z + my * q0q0 - _2q0mz * q.x + _2q1mx * q.y - my * q1q1 + my * q2q2 + _2q2 * mz * q.z - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q.y + _2q0my * q.x + mz * q0q0 + _2q1mx * q.z - mz * q1q1 + _2q2 * my * q.z - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.y + _2bz * q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.z - _4bz * q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.x + _2bz * q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.w - _4bz * q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q.w += qDot1 * dt;
	q.x += qDot2 * dt;
	q.y += qDot3 * dt;
	q.z += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	q.w *= recipNorm;
	q.x *= recipNorm;
	q.y *= recipNorm;
	q.z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRS::MadgwickAHRSupdateIMU(Quaternion_ &q,float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
	qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
	qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
	qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q.w;
		_2q1 = 2.0f * q.x;
		_2q2 = 2.0f * q.y;
		_2q3 = 2.0f * q.z;
		_4q0 = 4.0f * q.w;
		_4q1 = 4.0f * q.x;
		_4q2 = 4.0f * q.y;
		_8q1 = 8.0f * q.x;
		_8q2 = 8.0f * q.y;
		q0q0 = q.w * q.w;
		q1q1 = q.x * q.x;
		q2q2 = q.y * q.y;
		q3q3 = q.z * q.z;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q.z - _2q1 * ax + 4.0f * q2q2 * q.z - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q.w += qDot1 * dt;
	q.x += qDot2 * dt;
	q.y += qDot3 * dt;
	q.z += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	q.w *= recipNorm;
	q.x *= recipNorm;
	q.y *= recipNorm;
	q.z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float MadgwickAHRS::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
MadgwickAHRS AHRS;
//====================================================================================================
// END OF CODE
//====================================================================================================