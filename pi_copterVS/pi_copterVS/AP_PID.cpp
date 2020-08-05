// 
// 
//#include <math.h>


#include "AP_PID.h"  
 
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
	RC = 1.0f / (2.0f * (float)M_PI*fCut);
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
		
		derivative = _last_derivative +	(delta_time / (RC + delta_time)) * (derivative - _last_derivative);

		// update state
		_last_error = error;
		_last_derivative = derivative;

		// add in derivative component

		output += _kd * derivative;
		
	}
	
	// Compute integral component if time has elapsed
	if ((_ki != 0) && (delta_time > 0)) {
		_integrator += (error * _ki) *  delta_time;

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
