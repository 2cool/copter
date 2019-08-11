// GPS.h
#include "Location.h"
#ifndef _GPS_h
#define _GPS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Location.h"






class GPSClass
{
 protected:
	
 public:
	 
	 float bearing;
	
	void init();
	void loop();
	LocationClass loc;

	
};

extern GPSClass GPS;

#endif

