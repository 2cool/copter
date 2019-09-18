// Stabilization.h

#ifndef _STABILIZATION_h
#define _STABILIZATION_h


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "AP_PID.h"
#include "define.h"
#include "mpu.h"
class StabilizationClass{


#define SPEED_X_SPEED 0
#define SPEED_Y_SPEED 1
#define SPEED_Z_PID 2

private:
	
	void max_speed_limiter(double &x, double &y);
	void dist2speed(double &x, double &y);
	void speed2dist(double &x, double &y);

	
	double SPEED_Z_CF, SPEED_XY_CF;

	double mc_z,mc_x,mc_y,d_speedX,d_speedY;
	float dist2speed_XY;
	float alt2speedZ;
	double accxy_stab(double dist, double maxA, double timeL);
	double accxy_stab_rep(double speed, double maxA, double timeL);
	//double throttle;
#define STAB_PIDS 3
	AP_PID pids[STAB_PIDS];
	double max_z_integrator;

	double needXR, needYR, needXV, needYV;

	void set_acc_xy_speed_kp(const double f){ pids[SPEED_X_SPEED].kP(f);	pids[SPEED_Y_SPEED].kP(f); }
	void set_acc_xy_speed_kI(const double f){ pids[SPEED_X_SPEED].kI(f);	pids[SPEED_Y_SPEED].kI(f); }
	void set_acc_xy_speed_kD(const double f){ pids[SPEED_X_SPEED].kD(f,3);	pids[SPEED_Y_SPEED].kD(f,3); }
	void set_acc_xy_speed_imax(const double f){ pids[SPEED_X_SPEED].imax(-f,f);	pids[SPEED_Y_SPEED].imax(-f,f); }
	
	
public:
	void setMaxAng();
	void setMinMaxI_Thr();
	void setNeedPos2Home();
	void add2NeedPos(double speedX, double speedY, double dt);
	void setNeedPos(double x, double y);
	void setNeedLoc(long lat, long lon, double &x, double &y);
	void fromLoc2Pos(long lat, long lon, double &x, double &y);
	double get_dist2goal();
	//void set_XY_2_GPS_XY();
	void  resset_z();
	void  resset_xy_integrator();
	//double getAltitude() { return sZ; }
	double getSpeed_Z(const double dist){
		return constrain(dist*alt2speedZ, max_speedZ_M, max_speedZ_P);
	}

	double getDist_Z(double speed){
		return speed/alt2speedZ;
	}

	double getSpeed_XY(double dist){		
		return dist* dist2speed_XY;
	}
	double getDist_XY(double speed){	
		return speed/ dist2speed_XY;
	}




	void setDefaultMaxSpeeds();
	


	float max_speedZ_P,max_speedZ_M,max_speed_xy;
	double Z();
	void XY(float &xF, float&yF);

	void init();
	string get_z_set();
	string get_xy_set();
	void setZ(const float  *ar);
	void setXY(const float  *ar);


};
extern StabilizationClass Stabilization;
#endif

