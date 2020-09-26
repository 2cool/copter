#pragma once
class ANG_PID
{

	enum PID{ _X = 0, _Y = 2, LOW = 0, HIGHT = 1 };
public:

	ANG_PID();

	float 	*get_pid(const float errorX, const float errorY, const float delta_time);

	/// Reset the PID integrator
	///
	void	reset_integrators(const bool l, const bool h);
	float* get_integrator();
	void  set_integrator(const float *i);
	void	kP(const float v) { _kp = v; }
	void	set_kI(const float l, const float h) { _ki[LOW] = l; _ki[HIGHT] = h; }
	void    set_kI_max(const float l, const float h) { _imax[LOW] = l; _imax[HIGHT] = h; }

	float    get_kI_max_low() { return _imax[LOW]; }
	float    get_kI_max_hight() { return _imax[HIGHT]; }

	void    set_kI_max_low(const float l) { _imax[LOW] = l; }
	void    set_kI_max_hight(const float h) { _imax[HIGHT] = h; }

	float	kP() { return _kp; }
	void    hi_2_error_max_diff(const float ang) { auto_reset_v = ang; };
	float	get_kI_hight() { return _ki[HIGHT]; }
	float	get_kI_low() { return _ki[LOW]; }
	void	set_kI_hight(const float a) { _ki[HIGHT]=a; }
	void	set_kI_low(const float a) { _ki[LOW] = a; }

private:
	float				auto_reset_v;
	float				_kp;
	float				_ki[2];
	float				_imax[2];
	float				_integrator[4];		///< integrator value


};


