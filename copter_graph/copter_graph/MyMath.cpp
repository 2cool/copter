#include "stdafx.h"
#include "MyMath.h"

double homeLat = 0, homeLon = 0;

#define RAD2GRAD 57.29578f
#define GRAD2RAD 0.0174533f
#define DELTA_ANGLE_C 0.001f
#define DELTA_A_RAD (DELTA_ANGLE_C*GRAD2RAD)
#define DELTA_A_E7 (DELTA_ANGLE_C*10000000)
void MyMath::sin_cos(double &x, double &y, const double lat, const double lon, const double lat2, const double lon2) {
	double rll = (lon2 - lon);
	double rlat = (lat);
	double rlat2 = (lat2);

	y = (double)(sin(rll)*cos(rlat2));
	x = (double)(cos(rlat)*sin(rlat2) - sin(rlat)*cos(rlat2)*cos(rll));

}
double MyMath::bearing_(const double lat, const double lon, const double lat2, const double lon2) {
	double x, y;
	sin_cos(x, y, lat, lon, lat2, lon2);
	return (double)atan2(y, x);  //minus и как у гугла

}

double MyMath::distance_(const double lat, const double lon, const double lat2, const double lon2) {
	double R = 6371000;
	double f1 = (lat);
	double f2 = (lat2);
	double df = (lat2 - lat);
	double dq = (lon2 - lon);

	double a = (double)(sin(df / 2)*sin(df / 2) + cos(f1)*cos(f2)*sin(dq / 2)*sin(dq / 2));
	return (double)(R * 2 * atan2(sqrt(a), sqrt(1 - a)));

}

MyMath::MyMath()
{
}


MyMath::~MyMath()
{
}
double sX = 0, sY = 0,speedX=0,speedY=0;
void MyMath::fromACC2S(float yaw, float _ax, float _ay, double dt, double &x, double &y) {

	double cosYaw = cos(yaw*GRAD2RAD);
	double sinYaw = sin(yaw*GRAD2RAD);

	const float ax = (-cosYaw*_ax + sinYaw*_ay);
	const float ay = (-cosYaw*_ay - sinYaw*_ax);

	sX += dt*(speedX + ax*dt*0.5f);
	speedX += (ax*dt);
	sY += dt*(speedY + ay*dt*0.5f);
	speedY += (ay*dt);

	x = sX;
	y = sY;
}


void MyMath::fromLatLon2XY(int lat_, int lon_, double &x, double &y) {


	double bearing, distance;

#ifdef DEBUG_MODE
	//printf("upd\n");
	//Debug.dump(lat_, lon_, 0, 0);
#endif
	
	double lat = 1.74532925199433e-9 * (double)lat_;  //radians
	double lon = 1.74532925199433e-9 * (double)lon_;

	if (homeLat == 0 || homeLon == 0) {
		homeLat = lat;
		homeLon = lon;
	}
	
	bearing = bearing_(lat, lon, homeLat, homeLon);
	distance = distance_(lat, lon, homeLat, homeLon);
	//	Debug.dump(lat_, lon_, lat + 0.01*GRAD2RAD, lon + 0.01*GRAD2RAD);
	//	Debug.dump(lat, lon, distance, bearing);
	y = distance*sin(bearing);
	x = distance*cos(bearing);

}

MyMath mymath;