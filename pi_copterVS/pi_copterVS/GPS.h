// GPS.h

#ifndef _GPS_h
#define _GPS_h

#include "WProgram.h"
#include "Location.h"
#include "define.h"





class GPSClass
{
 protected:
	
 public:
	 
	 float bearing;
	
	bool init();
	bool loop();
	LocationClass loc;

	
};

extern GPSClass GPS;

#endif

