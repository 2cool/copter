// debug.h

#ifndef _DEBUG_h
#define _DEBUG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "mpu.h"

class DebugClass
{
 protected:
#define MAXARR 10
	 int i;
	 float ar[MAXARR][3];


	 void graphic(const int n, const float x, const float y);
	 void graphic(const int n, const float x, const float y,const float z);
 public:
	 float sett[10];
	 string get_set(bool for_save = false);
	 void set(const float buf[]);
	 uint32_t escCalibr;
	 
	 float  lowest_altitude_to_fly;
	 

	 int n_debug;
	 void init();
	 void dump(const long f1=0, long f2=0, long f3=0, long f4=0);
	 void dump(const double f1, double f2=0, double f3=0, double f4=0);


	 void dump(const uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4);
	 void load(const uint8_t i, const float x, const float y);
	 void load(const uint8_t i, const float x, const float y,const float z);
	 void dump(bool thre=false);
	
};

extern DebugClass Debug;

#endif

