#include "ANG_PID.h"
#define RAD2GRAD 57.295779513082320876798154814105
#define GRAD2RAD 0.01745329251994329576923690768489
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


//-------------
void ANG_PID::to_max_ang(const float ang, float& angX, float& angY) {
	float k = sqrt(angX * angX + angY * angY);
	if (k > ang) {
		k = ang / k;
		angX *= k;
		angY *= k;
	}
}
ANG_PID::ANG_PID()
{	
	_ki=0;
	_imax=0;
	max_output=0;
	reset_integrators();
}

void	ANG_PID::reset_integrators() { 
		_integrator[0] = _integrator[1] = 0; 
}

void  ANG_PID::set_integrator(const float ix, const float  iy) {
	_integrator[0] = ix;
	_integrator[1] = iy;
}
void ANG_PID::get_integrator(float& ix, float& iy) {
	ix = _integrator[0];
	iy = _integrator[1];
}

float 	*ANG_PID::get_pid(const float errorX, const float errorY, const float delta_time){
		float *output = new float[2];
		output[0] = errorX * _kp;
		output[1] = errorY * _kp;
		if (_imax!=0 && _ki!=0){
			_integrator[0] += (errorX * _ki) * delta_time;
			_integrator[1] += (errorY * _ki) * delta_time;
			to_max_ang(_imax, _integrator[0], _integrator[1]);
			if (max_output > 0) {
				float ox = _integrator[0] + output[0];
				float oy = _integrator[1] + output[1];
				to_max_ang(max_output, ox, oy);
				_integrator[0] = ox - output[0];
				_integrator[1] = oy - output[1];
			}
			output[0] += _integrator[0];
			output[1] += _integrator[1];
		}
		return output;
	}


