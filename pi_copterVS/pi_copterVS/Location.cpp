


//#define REAL_GPS
// 
// 
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
//#include <thread>

#include "Location.h"
#include "define.h"
#include "debug.h"
#include "Stabilization.h"
#include "Autopilot.h"
#include "Log.h"
#include "Telemetry.h"

#define DELTA_ANGLE_C 0.001
#define DELTA_A_RAD (DELTA_ANGLE_C*GRAD2RAD)
#define DELTA_A_E7 (DELTA_ANGLE_C*10000000)

#define SPEED_ERROR 60

double LocationClass::from_lat2X(const double lat) {
	return lat * kd_lat_;
}
double LocationClass::form_lon2Y(const double lon) {
	return lon * kd_lon_;
}


double LocationClass::from_X2Lat(const double x) {
	return x * r_kd_lat;
}


double LocationClass::from_Y2Lon(const double y) {
	return y * r_kd_lon;
}


uint8_t bugs = 0;


void LocationClass::print_gps_error() {
	cout << "gps error " << lat_ << " : " << lon_ << " " << altitude << " " << accuracy_hor_pos_ << " " << accuracy_ver_pos_ << " " << dt << ", " << last_gps_data__time << endl;;
	mega_i2c.beep_code(B_GPS_ACCURACY_E);
	Telemetry.addMessage(e_GPS_ERROR);
}


void LocationClass::fromLoc2Pos(const long &lat, const long &lon, double&x, double&y) {
	get(lat, lon, x, y);

//	y = form_lon2Y((_lon_zero - lon));
	//x = from_lat2X((_lat_zero - lat));
}
void LocationClass::xy(){
	double tx, ty;
	get(tx, ty);
	if (dY == 0 && dX == 0 && old_alt == 0) {
		dY = ty;
		dX = tx;
		old_alt = altitude;
	}
	double tspeedY = (ty - dY) *rdt;

	if (abs(tspeedY) > SPEED_ERROR) {
		bugs++;
		print_gps_error();
		return;
	}
	else {
		dY = ty;
		speedY = tspeedY;
	}
	//t = from_lat2X((_lat_zero - lat_));

	double tspeedX = (tx - dX) * rdt;
	if (abs(tspeedX) > SPEED_ERROR) {
		bugs++;
		print_gps_error();
		return;
	}
	else {
		dX = tx;
		speedX = tspeedX;
	}
	//update z
	double tsz = (altitude - old_alt)*rdt;
	
	if (abs(tsz) > SPEED_ERROR) {
		bugs++;
		print_gps_error();
		return;
	}
	else {
		old_alt = altitude;
		speedZ = tsz;
	}

#ifdef XY_SAFE_AREA
	if (Autopilot.motors_is_on() && sqrt(x2home*x2home+y2home*y2home)>XY_SAFE_AREA)
		Autopilot.control_falling(e_OUT_OF_PER_H);
#endif
}

void LocationClass::get(double& dx, double& dy) {
	get(lat_, lon_, dx, dy);
}
void LocationClass::get(const long &l_lat, const long &l_lon, double& dx, double& dy){
	const double lat = 1.74532925199433e-9 * (double)l_lat;  //radians
	const double lon = 1.74532925199433e-9 * (double)l_lon;
	const double bearing = bearing_(lat, lon, lat_zero, lon_zero);
	const double distance = distance_(lat, lon, lat_zero, lon_zero);
	dy = distance * sin(bearing);
	dx = distance * cos(bearing);
}
void LocationClass::update(){

	
	double bearing, distance;

#ifdef DEBUG_MODE
	//printf("upd\n");
	//Debug.dump(lat_, lon_, 0, 0);
#endif
	double lat = 1.74532925199433e-9 * (double)lat_;  //radians
	double lon = 1.74532925199433e-9 * (double)lon_;

	bearing = bearing_(lat, lon, lat + DELTA_A_RAD, lon + DELTA_A_RAD);
	distance = distance_(lat, lon, lat + DELTA_A_RAD, lon + DELTA_A_RAD);
//	Debug.dump(lat_, lon_, lat + 0.01*GRAD2RAD, lon + 0.01*GRAD2RAD);
//	Debug.dump(lat, lon, distance, bearing);
	double y = distance*sin(bearing);
	double x = distance*cos(bearing);
	
	//kd_lat = -x *0.00001;
	//kd_lon = -y *0.00001;

	kd_lat_ = x / (DELTA_A_E7);
	r_kd_lat = 1.0f / kd_lat_;
	kd_lon_ = y / (DELTA_A_E7);
	r_kd_lon = 1.0f / kd_lon_;

}
#define MAX_DIST2UPDATE 1000
void LocationClass::updateXY(){
	static double oldDist = MAX_DIST2UPDATE + MAX_DIST2UPDATE;
	double dx = (dX - x_from_zero_2_home);
	double dy = (dY - y_from_zero_2_home);
	dist2home = sqrt(dx*dx + dy*dy);
	double speed = abs(dist2home - shmPTR->dist2home) * rdt;
	shmPTR->dist2home = dist2home;
	if (speed > SPEED_ERROR) {
		bugs++;
		print_gps_error();
		return;
	}
	if (fabs(dist2home - oldDist) > MAX_DIST2UPDATE){
		oldDist = dist2home;
		update(); //for emulator
	}
	xy();
}
//////////////////////////////////////////////////////////////
void LocationClass::proceed(SEND_I2C *d) {
	bugs = 0;
	last_gps_data__time = millis_();
	dt = 0.001*(last_gps_data__time - old_time);
	dt = (dt < 0.2) ? 0.1 : 0.2;
	rdt = 1.0 / dt;
	old_time = last_gps_data__time;


#ifdef REAL_GPS
	accuracy_hor_pos_ = (d->hAcc > 99) ? 99 : d->hAcc;
	accuracy_ver_pos_ = (d->vAcc > 99) ? 99 : d->vAcc;
	if (accuracy_hor_pos_ < MIN_ACUR_HOR_POS_4_JAMM)
		last_gps_accuracy_ok = last_gps_data__time;
	
	altitude = 0.001*(double)d->height;
	lat_ = d->lat;
	lon_ = d->lon;
	//cout << lat_ << " " << lon_ <<" "<< (uint)accuracy_hor_pos_<< endl;
#else
	shmPTR->accuracy_hor_pos_ = accuracy_hor_pos_ = 1;
	shmPTR->accuracy_ver_pos_ = accuracy_ver_pos_ = 1;
	const long drand = last_gps_data__time & 0xf;
	shmPTR->gps_altitude_ = 15000 + drand;
	altitude = 0.001 * 15000;
	
	shmPTR->lat_ = lat_ = 479079700 + drand;
	shmPTR->lon_ = lon_ = 333320180 + drand;; //

	last_gps_accuracy_ok = last_gps_data__time;
	///shmPTR->lat_ = lat_ = 479079060;shmPTR->lon_ = lon_ = 333321560; //dvor
#endif

	if (_lat_zero == 0 && _lon_zero == 0 && accuracy_hor_pos_ <= Autopilot.min_hor_accuracy_2_start) {
		_lat_zero = lat_;
		_lon_zero = lon_;
		lat_zero = 1.74532925199433e-9 * (double)lat_;  //radians
		lon_zero = 1.74532925199433e-9 * (double)lon_;
		alt_zero = altitude;
	}

	if (_lat_zero != 0 || _lon_zero != 0)
		updateXY();





	if (bugs) {
		altitude = 0.001 * (double)shmPTR->gps_altitude_;
		accuracy_hor_pos_= shmPTR->accuracy_hor_pos_;
		accuracy_ver_pos_ = shmPTR->accuracy_ver_pos_;
		lat_ = shmPTR->lat_;
		lon_ = shmPTR->lon_;
		dist2home = shmPTR->dist2home;
		errors++;
		if (errors >= 5) {
			Autopilot.control_falling("GPS");
		}
	}
	else {
		shmPTR->gps_altitude_ = d->height;
		shmPTR->accuracy_hor_pos_ = accuracy_hor_pos_;
		shmPTR->accuracy_ver_pos_ = accuracy_ver_pos_;
		shmPTR->lat_ = lat_;
		shmPTR->lon_ = lon_;
		shmPTR->dist2home = (float)dist2home;
		shmPTR->speedY = speedY;
		shmPTR->speedX = speedX;
		shmPTR->speedZ = speedZ;
		errors = 0;
	}


	if (Log.writeTelemetry) {
		Log.block_start(LOG::GPS_SENS);
		Log.loadSEND_I2C(d);
		Log.loadFloat((float)dX);
		Log.loadFloat((float)speedX);
		Log.loadFloat((float)dY);
		Log.loadFloat((float)speedY);
		Log.block_end();
	}


}
/*
void LocationClass::add2NeedLoc(const double speedX, const double speedY, const double dt){
	//double t = (add_lat_need+= from_X2Lat(speedX*dt));
	//add_lat_need -= t;


	lat_needR_ -= from_X2Lat(speedX*dt);
	lat_needV_ = lat_needR_ - from_X2Lat(Stabilization.getDist_XY(speedX));
	//t = (add_lon_need += from_Y2Lon(speedY*dt));
	//add_lon_need -= t;
	lon_needR_ -= from_Y2Lon(speedY*dt);
	lon_needV_ = lon_needR_ - from_Y2Lon(Stabilization.getDist_XY(speedY));
}
*/


//lat 0.001 грудус = 111.2 метра
//lon 0.001 грудус = 74.6 метра

//lat 899.2805755396
//lon 1340.482573727

int LocationClass::init(){
#ifndef FLY_EMULATOR

	
#endif
	errors = 0;
	bugs = 0;
	dist2home = 0;
	x_from_zero_2_home = y_from_zero_2_home = 0;
	mspeedx =  mspeedy = 0;
	old_time = last_gps_data__time = last_gps_accuracy_ok = 0;
	dt = 0.1f;
	speedX = speedY = 0;
	_lon_zero = _lat_zero = 0;
	accuracy_hor_pos_ = 99;
	accuracy_ver_pos_ = 99;
	altitude = 0;
	lat_ = 0;  //radians
	lon_ = 0;
	dt = 0.1;
	rdt = 10;
	speedX = speedY = speedZ = 0;
	cout << "loc init\n";
	return 0;
	dX = dY = old_alt = 0;
}

void LocationClass::setHomeLoc(){
	x_from_zero_2_home = dX;
	y_from_zero_2_home = dY;
	shmPTR->lat_home = lat_;// lat_home;
	shmPTR->lon_home = lon_;// lon_home;
	old_alt=startAlt = altitude;

	//Debug.dump(lat_, lon_, 0, 0);
//	dX=dY=speedZ=speedX = speedY = x2home = y2home = accX=accY=accZ=0;
//	lat_needR_ = lat_needV_ = lat_home;
//	lon_needR_ = lon_needV_ = lon_home;

	//setNeedLoc2HomeLoc();
}

/*
void LocationClass::setNeedLoc(long lat, long lon){
	lat_needR_ = lat_needV_ = lat;
	lon_needR_ = lon_needV_ = lon;
	xy(false);
	//set_cos_sin_dir();

}


void LocationClass::setNeedLoc2HomeLoc(){
	//setNeedLoc(lat_home, lon_home);
	lat_needR_ = lat_needV_ = lat_home;
	lon_needR_ = lon_needV_ = lon_home;
	xy(false);
}
*/
double LocationClass::set_cos_sin_dir(){

//	double angle = atan2(dY, dX);

//	dir_angle_GRAD = angle*RAD2GRAD;
	//ErrorLog.println(angle*RAD2GRAD);
//	cosDirection = fabs(cos(angle));
//	sinDirection = fabs(sin(angle));
	return 0;
}

double LocationClass::bearing_(const double lat, const double lon, const double lat2, const double lon2){
	double x, y;
	sin_cos(x, y, lat, lon, lat2, lon2);
	return atan2(y, x);  //minus и как у гугла

}
void LocationClass::sin_cos(double &x, double &y, const double lat, const double lon, const double lat2, const double lon2){
	double rll = (lon2 - lon);
	double rlat = (lat);
	double rlat2 = (lat2);

	y = (sin(rll)*cos(rlat2));
	x = (cos(rlat)*sin(rlat2) - sin(rlat)*cos(rlat2)*cos(rll));

}
double LocationClass::distance_(const double lat, const double lon, const double lat2, const double lon2){
	double R = 6371000;
	double f1 = (lat);
	double f2 = (lat2);
	double df = (lat2 - lat);
	double dq = (lon2 - lon);

	double a = (sin(df / 2)*sin(df / 2) + cos(f1)*cos(f2)*sin(dq / 2)*sin(dq / 2));
	return R * 2 * atan2(sqrt(a), sqrt(1 - a));

}



