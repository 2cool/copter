// Telemetry.h

#ifndef _TELEMETRY_h
#define _TELEMETRY_h


#include "WProgram.h"
#include "define.h"
#include "debug.h"


enum {	NO_MESSAGE = 0, START_ROTTATION = 1, STOP_ROTTATION};

enum { T_TEMP = 0, T_PRES = 1, T_LAT = 3, T_LON = 7, T_GPS_HEIGHT = 11, T_GPS_HEADING = 13, T_HEADING = 14, T_ROL_X = 15, T_PITCH_Y = 16, T_BATARY = 17, T_RECIVED_COUNTER = 20,T_MES_CODE = 21,T_SPEED };

class TelemetryClass
{
 protected:
	 float battery_charge;
	 uint8_t *buf;

	 void loadBUF32(int &i, int32_t val);
	 void loadBUF16(int &i, int16_t val);
	 void loadBUF(int &i,  const float fval);
	 void loadBUF8(int &i, const float val);

	 bool newGPSData;
	
	 float SN;
	 
	// uint8_t inner_clock_old_sec;
	 int get_saved_voltage();
	 int32_t next_battery_test_time;
	  uint32_t pressure;
	 float voltage,voltage_at_start;
	 float m_current[4];
	 
	 string message;
	 bool update_buf();
	 float full_power;
 public:
	 uint8_t cpu_temp;
	 int on_power_time = 0, total_time=0;
	 void save_voltage();
	 float get_bat_capacity();
	 float get_full_power() { return full_power; }
	 void set_bat_capacity(float a_ch);
	 uint8_t no_time_cnt = 0;
	 int update_voltage();
	 float powerK;

	 void clearMessage(){ message = ""; }
	 void addMessage(const string msg, bool and2sms=false);
	 string getMessage(){ return message; }
	 void getSettings(int n);
	 bool minimumTelemetry;
	 bool low_voltage,voltage50P;
	 
	 uint8_t lov_voltage_cnt;



	 int check_time_left_if_go_to_home();
	 int fly_time_left();
	 bool testBatteryVoltage();

	 void init_();

	 bool loop();

	 float get_voltage(){ return voltage; }
	 float get_current(const int i) { return m_current[i&3]; }
	 int get_voltage4one_cell();
	
	
	// string getSport();
	 
	

};

extern TelemetryClass Telemetry;

#endif

/*


Charge voltage 	  					3.3V 	3.5V 	3.6V 	3.7V 	3.8V 	3.9V 	4.0V 	4.05V 	4.1V 	4.15V 	4.2V 	4.25V* 	4.3V*
Percentage of 4.2V capacity 	  	0% 		2.9% 	5.0% 	8.6% 	36% 	62% 	73% 	83% 	89% 	94% 	100% 	105% 	106%



*/