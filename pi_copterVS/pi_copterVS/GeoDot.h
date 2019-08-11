// GeoDot.h

#ifndef _GEODOT_h
#define _GEODOT_h
#include <inttypes.h>
class GeoDotClass
{

 protected:

 public:

	 int32_t lat, lon;
	 int16_t altitude;
	 int8_t  direction, cameraAngle,turnOutAngle;
	 uint8_t timer;
	 float speedXY, speedZ;

	void init();
};

extern GeoDotClass GeoDot;

#endif

