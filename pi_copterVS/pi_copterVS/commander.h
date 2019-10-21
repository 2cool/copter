// commander.h

#ifndef _COMMANDER_h
#define _COMMANDER_h



#include "define.h"
#include "WProgram.h"

#include "Autopilot.h"
class CommanderClass
{
 protected:
	
	

//	 bool ManualControl(int8_t *msg);//manual

//	 bool ButtonMessage(byte *msg);//button command

	 bool ButtonMessage(string);

	 
	 float throttle, yaw, yaw_offset, pitch, roll;
	

 public:
	
	 volatile bool ppp_inet, telegram_bot;
	
	 void setThrottle(const float t){ throttle = t; }
	 void setPitch(const float p){ pitch = p; }
	 void setRoll(const float r){ roll = r; }
	 float get_yaw_minus_offset();
	 float getYaw();
	 float getThrottle(){ return throttle; }
	 float getPitch();
	 float getRoll();
	 void data_reset();
	 string get_set();
	 void set(const float buf[]);
	
	 bool ret;
	//short recived_counter;
	
	

	 bool init();
	 bool input();
	 void controls2zero();

};
	

extern CommanderClass Commander;

#endif

