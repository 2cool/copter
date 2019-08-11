#pragma once
#include <math.h>
class MyMath
{
public:
	MyMath();
	~MyMath();
	void fromLatLon2XY(int lat, int lon, double &x, double &y);

	void fromACC2S(float yaw, float accX, float accY, double dt, double &x, double &y);


	void sin_cos(double &x, double &y, const double lat, const double lon, const double lat2, const double lon2);
	double bearing_(const double lat, const double lon, const double lat2, const double lon2);

	double distance_(const double lat, const double lon, const double lat2, const double lon2);






};

extern MyMath mymath;