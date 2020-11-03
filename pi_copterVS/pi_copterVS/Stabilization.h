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

	void max_speed_limiter(float &x, float &y);
	void dist2speed(const float dx, const float dy, float &speed_x, float &speed_y);
	void speed2dist(const float speedX, const float speedY, float &dx, float &dy);

	
	float SPEED_Z_CF, SPEED_XY_CF;
	float hor_speed_kp;
	float hor_acc_kd;

	float hor_pos_ki;
	float hor_pos_ki_max;
	float hor_pos_kp; 
	float hor_speed_kd;
	float z_error;
	float dist2speed_XY;
	float alt2speedZ;
	float z_kD;

#define STAB_PIDS 3
	AP_PID pid_ver;
	float max_z_integrator;

	float needXR, needYR, needXV, needYV;
	   
	float def_max_speedXY, current_max_speed_xy;
	float def_max_speedZ_P, def_max_speedZ_M,current_max_speedZ_P,current_max_speedZ_M, min_stab_Z_speed;
	float allowance;
public:
	float get_allowance() { return allowance; }
	void to_max_ang(const float ang, float& angX, float& angY);
	void set_max_speed_hor(float& s, bool only_test = false);
	void set_max_sped_ver(float &ps, float &ns, bool only_test = false);
	float get_def_max_speedXY() { return def_max_speedXY; }
	float get_def_max_speedZ_M() { return def_max_speedZ_M; }
	float get_max_speedZ_M_4_go_to_home() { return max(MIN_SPEED_TO_GO_TO_HOME_Z, -def_max_speedZ_M); }
	float get_max_speedZ_P_4_go_to_home() { return max(MIN_SPEED_TO_GO_TO_HOME_Z, def_max_speedZ_P); }

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

	void Hor_position(float& xF, float& yF);
	void Hor_speed(float &xF, float&yF);

	void init();
	string get_z_set();
	string get_xy_set();
	void setZ(const float  *ar);
	void setXY(const float  *ar);


};
extern StabilizationClass Stabilization;
#endif

