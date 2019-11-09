#pragma once

#include "graph.h"

class GPS_Loger
{
public:
	float startZ;
	double max_y, min_y, max_x, min_x;
	double gx2home, gy2home, gdX, gdY, gspeedX, gspeed, gspeedY, z,max_z,min_z;
	double gax, gay,yaw;
	double old_z = 0, gspeedZ, old_gspeeZ = 0, old_sz;
	double gx = 0, gy = 0, lat = 0, lon = 0;
	double r_lat, r_lon;
	int acurH, acurV;
	int init();
	int decode(byte buf[], int i, int len, bool rotate = false);
	int view(int &indexes, char buffer[], int &i);
};

extern GPS_Loger gps_log;