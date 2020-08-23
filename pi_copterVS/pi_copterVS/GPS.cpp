 
#include "Hmc.h"
#include "define.h"
#include "GPS.h"
#include <math.h>
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"
#include "Log.h"


bool GPSClass::init()
{	

#ifndef	FLY_EMULATOR
	//loc.last_gps_data_timed = 10;
#endif
	if (loc.init() == -1) {
		cout << "GPS ERROR\n";
		return false;
	}
	cout << "GPS INIT\n";
	return true;
}


#ifdef FLY_EMULATOR


int32_t gpsttime = 0;
//float distX = 0, distY = 0;
//float speedX = 0, speedY = 0;
float fdt = 0;
bool gps_foo1 = true;
int32_t tkskks = 0;

//#define FALSE_LAT 479001194
//#define FALSE_LON 332531662


//park gag
//#define FALSE_LAT 479059400
//#define FALSE_LON 333368000

//pole
//#define FALSE_LAT 479093270
//#define FALSE_LON 333304280

#define FALSE_LAT 479035710
#define FALSE_LON 333395120
//#define FALSE_LAT 478877891 
//#define FALSE_LON 333048704

long lat = FALSE_LAT;
long lon = FALSE_LON;




float fullX = 0, fullY = 0;
bool GPSClass::loop(){




	gpsttime = millis_()/100;
	
	if (loc.mseconds != gpsttime)
		loc.mseconds = gpsttime;
	else
		return false;
	shmPTR->accuracy_hor_pos_=loc.accuracy_hor_pos_ = 1;
	shmPTR->accuracy_ver_pos_=loc.accuracy_ver_pos_ = 1;
	shmPTR->gps_altitude_=loc.altitude = Emu.get_alt();





	loc.dt = 0.1;

	
	
	if (Autopilot.motors_is_on() == false){
		//speedX = speedY = 0;
		
	}

#ifdef FLY_EMULATOR
		if (Mpu.get_Est_Alt() <= 0){
			//speedX = speedY = 0;
	
			//distX = distY = 0;
		}
#endif


		//Debug.dump(lat*1000, lon*1000, loc.x2home, loc.y2home);
		

		

#ifndef FASLE_GPS_STILL
		lat = FALSE_LAT + (long)(loc.from_X2Lat(Emu.get_x()));
		lon = FALSE_LON + (long)(loc.from_Y2Lon(Emu.get_y()));
#endif

		if (loc._lat_zero == 0 && loc._lon_zero == 0) {
			loc._lat_zero = lat;
			loc._lon_zero = lon;
			loc.lat_zero = 1.74532925199433e-9 * (double)lat;  //radians
			loc.lon_zero = 1.74532925199433e-9 * (double)lon;
			//alt_zero = altitude;
		}



		shmPTR->lat_=loc.lat_ = lat;
		shmPTR->lon_=loc.lon_ = lon;
		loc.updateXY();

			if (Log.writeTelemetry) {

				SEND_I2C posllh;
				posllh.lat = lat;
				posllh.lon = lon;
				posllh.height = Emu.get_alt();


				Log.block_start(LOG::GPS_SENS);
				Log.loadSEND_I2C(&posllh);
				Log.block_end();
			}
			return true;
}

#else



SEND_I2C g_data;

bool GPSClass::loop(){

	const int32_t _ct = millis_();

	int ret = mega_i2c.get_gps(&g_data);
	
	if (ret == -1) {
		Telemetry.addMessage(e_GPS_ERROR);
		cout << "gps right write error  , uptime=" << _ct << ",msec. last upd=" << loc.last_gps_data__time << "msec. \n";
		mega_i2c.beep_code(B_I2C_ERR);
		return true;
	}else if (ret>0)
		loc.proceed(&g_data);
	if ( _ct - loc.last_gps_data__time > 300 && loc.last_gps_data__time){
		Telemetry.addMessage(e_GPS_ERROR);
		cout << "gps update error  , uptime=" << (float)_ct*0.001f << ",sec. dt=" << _ct - loc.last_gps_data__time << "msec. \n";
		mega_i2c.beep_code(B_GPS_TOO_LONG);
	}
	if (Autopilot.motors_is_on() && (_ct - loc.last_gps_accuracy_ok) > NO_GPS__DATA && loc.last_gps_accuracy_ok) {
#ifndef DEBUG
		cout << "gps jamming, accHP="<<(int)loc.accuracy_hor_pos_<<"  , uptime=" << _ct << ",msec. last upd=" << loc.last_gps_data__time << "msec. \n";
#endif		
	}	

	return true;
}
#endif

GPSClass GPS;

