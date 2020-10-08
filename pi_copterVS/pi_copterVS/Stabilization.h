// Stabilization.h

#ifndef _STABILIZATION_h
#define _STABILIZATION_h


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include "AP_PID.h"
#include "ANG_PID.h"
#include "define.h"
#include "mpu.h"
class StabilizationClass{


private:
	void calculate_wind_i(float world_ang[]);
	void max_speed_limiter(float &x, float &y);
	void dist2speed(float &x, float &y);
	void speed2dist(float &x, float &y);

	
	float SPEED_Z_CF, SPEED_XY_CF;

	float z_error,x_error,y_error;
	float dist2speed_XY;
	float alt2speedZ;
	float xy_kD;
	float z_kD;

#define STAB_PIDS 3
	ANG_PID pid_hor;
	AP_PID pid_ver;
	float max_z_integrator;

	float needXR, needYR, needXV, needYV;
	   
	float def_max_speedXY, min_stab_XY_speed, current_max_speed_xy;
	float def_max_speedZ_P, def_max_speedZ_M,current_max_speedZ_P,current_max_speedZ_M, min_stab_Z_speed;
public:
	void to_max_ang(const float ang, float& angX, float& angY);
	void set_max_speed_hor(float& s, bool only_test = false);
	void set_max_sped_ver(float &ps, float &ns, bool only_test = false);
	float get_def_max_speedXY() { return def_max_speedXY; }
	float get_min_stagXY() { return min_stab_XY_speed; }
	float get_def_max_speedZ_M() { return def_max_speedZ_M; }
	float get_max_speedXY_4_go_to_home() { return max(MIN_SPEED_TO_GO_TO_HOME_XY, min_stab_XY_speed); }
	float get_max_speedZ_M_4_go_to_home() { return max(MIN_SPEED_TO_GO_TO_HOME_Z, -def_max_speedZ_M); }
	float get_max_speedZ_P_4_go_to_home() { return max(MIN_SPEED_TO_GO_TO_HOME_Z, def_max_speedZ_P); }
	void setMaxAngels();
	void setMinMaxI_Thr();
	void setNeedPos2Home();
	void add2NeedPos(float speedX, float speedY, float dt);
	void setNeedPos(float x, float y);
	void setNeedLoc(long lat, long lon, double &x, double&y);
	void fromLoc2Pos(long lat, long lon, double &x, double&y);
	float get_dist2goal();
	//void set_XY_2_GPS_XY();
	void  resset_z();
	void  resset_xy_integrator();
	//float getAltitude() { return sZ; }
	float getSpeed_Z(const float dist){
		return constrain(dist*alt2speedZ, current_max_speedZ_M, current_max_speedZ_P);
	}

	float getDist_Z(float speed){
		return speed/alt2speedZ;
	}

	float getSpeed_XY(float dist){		
		return dist* dist2speed_XY;
	}
	float getDist_XY(float speed){	
		return speed/ dist2speed_XY;
	}



	void setDefaultMaxSpeeds4Return2HOME();
	void setDefaultMaxSpeeds();
	


	
	float Z();
	void XY(float &xF, float&yF);

	void init();
	string get_z_set();
	string get_xy_set();
	void setZ(const float  *ar);
	void setXY(const float  *ar);


};
extern StabilizationClass Stabilization;
#endif

