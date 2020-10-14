#pragma once
#include "math.h"
class ANG_PID
{

public:

	ANG_PID();

	float 	*get_pid(const float errorX, const float errorY, const float delta_time);

	/// Reset the PID integrator
	///
	void	reset_integrators();
	void   get_integrator(float &ix, float&iy);
	float  get_integratorX() { return _integrator[0]; }
	float  get_integratorY() { return _integrator[1]; }
	void  set_integrator(const float ix, const float  iy);
	void set_max_output(const float a) { max_output = a; }
	float get_max_output() { return max_output; }
	void	kP(const float v) { _kp = v; }
	void	set_kI(const float v) { _ki=v; }
	void    set_kI_max(const float v) { _imax = v;  }
	float    get_kI_max() { return _imax; }
	float	kP() { return _kp; }
	float	get_kI() { return _ki; }
	void	set_kI_low(const float a) { _ki = a; }
	void	to_max_ang(const float ang, float& angX, float& angY);
private:
	float				_kp;
	float				_ki;
	float				_imax;
	float				max_output;
	float				_integrator[2];		///< integrator value


};


