#include "stdafx.h"
#include "GPS_Loger.h"
#include "graph.h"

double z_cor = 101010;
FILE *klm;
int GPS_Loger::init() {
	//startZ = 122222220;
	//klm = fopen("d:/logs/klm.txt", "w");


	
	



	acurH = acurV = 99;




	return 0;// klm == 0 ? -1 : 0;
}








uint32_t def_mode = Z_STAB + XY_STAB;



//void Mpu::parser(byte buf[], int j, int len) {
int GPS_Loger::decode(/*SEND_I2C*p, */byte buf[], int i, int len, bool rotate ) {

	
	SEND_I2C* p = (SEND_I2C*)&buf[i];
	//if (p->hAcc < 10)
	//	p->hAcc = 10;
	//old_z = 0, gspeedZ, old_gspeeZ = 0
	static double first_alt = 0;


	z = 0.001*(double)p->height;
	if (z_cor == 101010) {
		z_cor = z;
		max_z = -10000; min_z = 10000;
	}






//	z_cor += (z - z_cor)*0.01;
	//z -= z_cor;
	max_z = max(max_z, z);
	min_z = min(min_z, z);



	if (first_alt == 0)
		first_alt = z;
//	z -= first_alt;
//	gspeedZ = (z - old_z) / 0.1;
//	old_z = z;
//	gaz = (gspeedZ - old_gspeeZ) / 0.1;
//	old_gspeeZ = gspeedZ;


	//33.4720041, 47.97157490000001, 96.20999999999999

	r_lat = p->lat * 0.0000001;
	r_lon = p->lon * 0.0000001;
	acurH = p->hAcc;
	acurV = p->vAcc;
	double n_lat = p->lat*0.0000001*GRAD2RAD;
	double n_lon = p->lon*0.0000001*GRAD2RAD;

	if (lat == 0 || lon == 0) {
		lat = n_lat;
		lon = n_lon;
	}

	if (len -  sizeof(SEND_I2C) >= 16) {
		i += sizeof(SEND_I2C);
		gx= *(float*)& buf[i]; i += 4;
		float speedX = *(float*)& buf[i]; i += 4;
		gy = *(float*)& buf[i]; i += 4;
		float speedY = *(float*)& buf[i]; i += 4;
		/*
		if (speedX == 0 && speedY == 0) {  //кастиль
			return 0;
		}*/
		gspeedX = speedX;
		gspeedY = speedY;
	}
	


	double distance = mymath.distance_(n_lat, n_lon, lat, lon);
	double bearing = mymath.bearing_(n_lat, n_lon, lat, lon);
	yaw = bearing*RAD2GRAD;
	static double old_distance = 0;

	//gy = (distance - old_distance)*sin(bearing);
	old_distance = distance;
	//gspeedY = (gy) / 0.1;


	//gx = (distance - old_distance)*cos(bearing);
	//gspeedX = (gx) / 0.1;


	//	lat = n_lat;
	//	lon = n_lon;

	
	

	/*
	if (startZ == 122222220)
		startZ = z;
	z -= startZ;
	i += 1 + sizeof(SEND_I2C);
	gx2home = *(float*)(&buffer[i]);
	gy2home = *(float*)(&buffer[i + 4]);
	gdX = *(float*)(&buffer[i + 8]);
	gdY = *(float*)(&buffer[i + 12]);
	//	gspeedX = *(float*)(&buffer[i + 16]);
	//	gspeedY = *(float*)(&buffer[i + 20]);
	gspeed = sqrt(gspeedX*gspeedX + gspeedY*gspeedY)*3.6;
	gax = *(float*)(&buffer[i + 24]);
	gay = *(float*)(&buffer[i + 28]);
	int vacc = p->vAcc;
	int hacc = p->hAcc;
	
	//if (dataI > 44467)
	{
		//std::wstring str = L" " + std::to_wstring(p->lon*0.0000001) + L"," + std::to_wstring(p->lat * 0.0000001) + L"," + std::to_wstring(p->height * 0.001) + L" ";
		//fwrite(str.c_str(), str.length(), 2, klm);
	}


//	i += 8 * 4;


	gy2home = distance *sin(bearing);
	gx2home = distance*cos(bearing);


	max_x = max(max_x, gx2home);
	min_x = min(min_x, gx2home);
	max_y = max(max_y, gy2home);
	min_y = min(min_y, gy2home);



	*/





	return 0;
}
int GPS_Loger::view(int &indexes, char buffer[], int &i) {

	return 0;
}

GPS_Loger gps_log;