#include "ANG_PID.h"

#include "math.h"
#define RAD2GRAD 57.295779513082320876798154814105
#define GRAD2RAD 0.01745329251994329576923690768489
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))





void to_max_ang(const float ang, float& angX, float& angY) {
	float k = sqrt(angX * angX + angY * angY);
	if (k > ang) {
		k = ang / k;
		angX *= k;
		angY *= k;
	}
}
//-------------

ANG_PID::ANG_PID()
{
}

void	ANG_PID::reset_integrators(const bool l, const bool h) { 
	if (l)
		_integrator[LOW + _X] = _integrator[LOW + _Y] = 0; 
	if (h)
		_integrator[HIGHT + _X] = _integrator[HIGHT + _Y] = 0; }

void  ANG_PID::set_integrator(const float* i) {
	_integrator[0] = i[0];
	_integrator[1] = i[1];
	_integrator[2] = i[2];
	_integrator[3] = i[3];
}
float * ANG_PID::get_integrator() {
	float* i = new float[4];
	i[0] = _integrator[0];
	i[1] = _integrator[1];
	i[2] = _integrator[2];
	i[3] = _integrator[3];
	return i;
}
float 	*ANG_PID::get_pid(const float errorX, const float errorY, const float delta_time)
	{
		float *output = new float[2];

		output[0] += errorX * _kp;
		output[1] += errorY * _kp;


		if (_ki[LOW] != 0) {//?
			_integrator[LOW + _X] += (errorX * _ki[LOW]) * delta_time;
			_integrator[LOW + _Y] += (errorY * _ki[LOW]) * delta_time;
			to_max_ang(_imax[LOW], _integrator[LOW + _X], _integrator[LOW + _Y]);
		}
		if (_ki[HIGHT] != 0) {//?
			_integrator[HIGHT + _X] += (errorX * _ki[HIGHT]) * delta_time;
			_integrator[HIGHT + _Y] += (errorY * _ki[HIGHT]) * delta_time;
			to_max_ang(_imax[HIGHT], _integrator[HIGHT + _X], _integrator[HIGHT + _Y]);

			if (auto_reset_v > 0) { 
				float dx = output[0] - _integrator[HIGHT + _X];
				float dy = output[1] - _integrator[HIGHT + _Y];
				to_max_ang(auto_reset_v, dx, dy);
				_integrator[HIGHT + _X] = output[0] - dx;
				_integrator[HIGHT + _Y] = output[1] - dy;
			}
		}
		output[0] += _integrator[LOW + _X] + _integrator[HIGHT + _X];
		output[1] += _integrator[LOW + _Y] + _integrator[HIGHT + _Y];

		return output;
	}


