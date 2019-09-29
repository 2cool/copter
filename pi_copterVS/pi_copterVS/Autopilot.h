// Autopilot.h

#ifndef _AUTOPILOT_h
#define _AUTOPILOT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "mi2c.h"
#include "MS5611.h"
#include "Location.h"
#include "AP_PID.h"


// in dicimetr
#define MAX_HEIGHT 15


#include "commander.h"
#include "mpu.h"
class AutopilotClass
{



 protected:
	 bool ignore_the_lack_of_internet_at_startup;
	 double dist2home_at_begin;
#ifdef FALL_IF_GPS_ALT_NOT_MATCH_BAROM_ALT_
	 float gps_alt_at_begin, bar_alt_at_begin;
#endif

	 bool howeAt2HOME;
	 float sens_z, sens_xy;

	 bool newData;
	 int32_t control_DeltaTime;
	 uint8_t go2homeIndex;
	 float f_go2homeTimer;
 
	 
	 float flyAtAltitude_V,flyAtAltitude_R;

	// bool motors_on, smart_ctrl;

	 float aPitch, aRoll,aYaw_;

	 uint32_t old_control_bits;
	 float throttle;
	 
	 uint32_t control_bits;


	 void log();

	 void smart_commander(const float dt);
	// void setNextGeoDot();
	 //void prog_loop();
	// float get_dist();
	 float lowest_height;
	 float fall_thr;
	 uint8_t hall_ok;

	 
 public:
	 
	 float min_hor_accuracy_2_start;
	 int32_t powerOnTime();
	 bool is_all_OK(bool print=false);
	 int reboot();
	 int shutdown();
	 int exit();
	 int32_t  time_at__start,old_time_at__start;
	 void gimBalRollCorrection();
	 bool busy() { return (control_bits & (MPU_ACC_CALIBR | MPU_GYRO_CALIBR | COMPASS_CALIBR)); }
	 volatile int32_t last_time_data__recived;
	 void setYaw(const float yaw){aYaw_ = yaw;}
	 float getGimbalPitch(){ return gimbalPitch; }
	 void control_falling(const string msg);
	 void gimBalPitchADD(const float add);
	 void gimBalRollADD(const float add);
	 void set_gimBalPitch(const float angle);

	 float corectedAltitude4tel();

	void reset_compas_motors_calibr_bit() {control_bits &= (~COMPASS_MOTOR_CALIBR);}
	void hall_test();
	bool motors_onState(){ return control_bits&MOTORS_ON; }
	bool z_stabState(){ return control_bits&Z_STAB; }
	bool xy_stabState(){ return control_bits&XY_STAB; }
	 bool go2homeState(){ return control_bits&GO2HOME; }
	 bool progState(){ return control_bits&PROGRAM; }
	 bool control_fallingState(){ return control_bits & CONTROL_FALLING; }
	 bool program_is_loaded() { return control_bits & PROGRAM_LOADED; }
	 void program_is_loaded(bool set);
	 bool set_control_bits(uint32_t bits);

	 float fly_at_altitude() { return flyAtAltitude_V; }
	 uint32_t get_control_bits(){ return control_bits; }
	 //uint8_t mod;  //режим работы 
	// bool falling(){ return ctrl_flag == CNTR_FALLING; }


	 float get_throttle(){ return throttle; }
	 float get_yaw(){ return aYaw_; }

	 
	 void add_2_need_altitude(float speed, const float dt);
	 void add_2_need_yaw(float speed, const float dt);
	// bool manualZ;
	
	string get_set(bool for_save=false);
	 void set(const float buf[]);

	
	 void clearSpeedCoreection(){ flyAtAltitude_V = flyAtAltitude_R; }

	// bool get_smart_cntr_flag(){ return smart_ctrl; }

	 float gimbalPitch,gimbalRoll;
	 float height_to_lift_to_fly_to_home = HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME;

	 bool going2HomeON(const bool hower);
	 bool going2HomeStartStop(const bool hower);
	 bool go2HomeProc(const float dt);


	 float get_Roll(){ return aRoll; }
	 float get_Pitch(){ return aPitch; }


	 
	 bool holdLocationStartStop();
	 bool holdLocation(const long lat, const long lon);
	 
	 bool holdAltitudeStartStop();
	 bool holdAltitude(float alt);
	 void	 set_new_altitude(float alt);
	 void init();
	 void loop();
	// uint8_t ctrl_flag;
	 bool motors_is_on(){ return control_bits & MOTORS_ON; }

	 bool start_stop_program(const bool stopHere);

	void connectionLost_();


	void calibration(); 

	//void set_height(const short h);
	bool off_throttle(const bool force,const string msg);
	bool motors_do_on(const bool start,const string msg);



private:

};

extern AutopilotClass Autopilot;

#endif

