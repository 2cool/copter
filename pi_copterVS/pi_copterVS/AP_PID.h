// AP_PID.h

#ifndef _AP_PID_h
#define _AP_PID_h






#include <inttypes.h>
#include <math.h>		// for fabs()

class AP_PID {
public:

	AP_PID();

	float 	get_pid(float error, float delta_time);

	/// Reset the PID integrator
	///
	void	reset_I();

	void	kP(const float v)		{ _kp = v; }
	void	kI(const float v)		{ _ki = v; }
	void	kD(const float v, const float fCut = 20.0);

	void	imax(const float min, const float max) { 
		_imax = max; 
		_imin = min; 
	}

	float	kP()			{ return _kp; }
	float	kI()			{ return _ki; }
	float	kD()			{ return _kd; }
	float	imax()			{ return _imax; }
	float   imin()			{ return _imin; }
	void   set_integrator(const float i);
	float	get_integrator() const	{ return _integrator; }

private:
	float				_fCut;
	float				 RC;
	float				_kp;
	float				_ki;
	float				_kd;
	float				_imax,_imin;

	float				_integrator;		///< integrator value
	float				_last_error;		///< last error for derivative
	float				_last_derivative; 	///< last derivative for low-pass filter

	/// Low pass filter cut frequency for derivative calculation.
	///
	/// 20 Hz becasue anything over that is probably noise, see
	/// http://en.wikipedia.org/wiki/Low-pass_filter.
	///
	
};

#endif
