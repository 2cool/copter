#include "stdafx.h"
#include "PID.h"
// 
// 
//#include <math.h>

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

AP_PID::AP_PID()
{
}

void   AP_PID::set_integrator(const float i) {
	_integrator = i;
	if (_integrator < _imin) {
		_integrator = _imin;
	}
	else if (_integrator > _imax) {
		_integrator = _imax;
	}
}

//float RC = 1.0f / (2.0f * (float)M_PI*20.0f);

void	AP_PID::kD(const float v, const float fCut) {
	RC = 1.0f / (2.0f * (float)M_PI * fCut);
	_kd = v;
}

float AP_PID::get_pid(float error, float delta_time)
{
	float output = 0;
	//float delta_time = (float)dt / 1000.0;

	// Compute proportional component
	output += error * _kp;
	// Compute derivative component if time has elapsed
	if ((_kd != 0) && (delta_time > 0)) {
		float derivative = (error - _last_error) / delta_time;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy

		derivative = _last_derivative +
			(delta_time / (RC + delta_time)) * (derivative - _last_derivative);

		// update state
		_last_error = error;
		_last_derivative = derivative;

		// add in derivative component

		output += _kd * derivative;

	}

	// Compute integral component if time has elapsed
	if ((_ki != 0) && (delta_time > 0)) {
		_integrator += (error * _ki) * delta_time;

		if (_integrator > _imax) {
			_integrator = _imax;
		}
		else {
			if (_integrator < _imin)
				_integrator = _imin;
		}

		/*
		if (_integrator < -_imax) {
			_integrator = -_imax;
		}
		else if (_integrator > _imax) {
			_integrator = _imax;
		}
		*/
		output += _integrator;
	}
	return output;
}

void
AP_PID::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	_last_derivative = 0;
}
