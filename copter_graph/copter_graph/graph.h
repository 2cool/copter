#pragma once

#include <windows.h>
#include <objidl.h>
#include <gdiplus.h>
#include <stdio.h>
#include <math.h>
#include <iostream> 
#include <string> 
#include "MyMath.h"
#include "G.h"

using namespace Gdiplus;
#pragma comment (lib,"Gdiplus.lib")

#define RAD2GRAD 57.29578f
#define GRAD2RAD 0.0174533f




#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

enum {
	MOTORS_ON = 1, CONTROL_FALLING = 2, Z_STAB = 4, XY_STAB = 8, GO2HOME = 0x10, PROGRAM = 0x20, COMPASS_ON = 0x40, HORIZONT_ON = 0x80,
	MPU_ACC_CALIBR = 0x100, MPU_GYRO_CALIBR = 0x200, COMPASS_CALIBR = 0x400, COMPASS_MOTOR_CALIBR = 0x800, RESETING = 0x1000, GIMBAL_PLUS = 0x2000, GIMBAL_MINUS = 0x4000
};


enum {
	PITCH,ROLL,YAW,GPS_YAW,
	GYRO_PITCH,GYRO_ROLL,GYRO_YAW,
	ACCX,ACCY,ACCZ,
	SX,SY,SZ, SPEED_X, SPEED_Y,SPEED_Z,
	GSX,GSY,GSPEED_X,GSPEED_Y,

	BAR_ALT, BAR_SPEED,GPS_ALT,


	C_PITCH,C_ROLL,C_YAW,
	MI0,MI1,MI2,MI3,VOLTAGE,
	THROTTLE, F0, F1, F2, F3,
	TIME,DT,

	
	ROTATE, FILTER, CONTROL_BITS,  ALL_ELEMENTS
};





struct SensorsData {
	double sd[ALL_ELEMENTS];
	
};

struct MODE {
	int mode, index;
};
#define G 9.8
#define MAX_ACC 7










struct SEND_I2C {

	int32_t lon;
	int32_t lat;
	int32_t height;
	uint8_t hAcc;
	uint8_t vAcc;
};



#include "G.h"

class Graph
	
{
private:

	


	void correct_c_pitch_c_roll(int p);

	uint16_t bat;
	uint16_t mi[4];

	int time_index;
	int gpsI= -1;
	void filter(float src, int dataI, int elementi,float max=4000);
	float cf;
	SensorsData *sensors_data;


	int decode_Log();
	char * buffer;
	int  sensorsData;
	long lSize;


	void drawModes(Graphics &g, RectF rect);
	void draw(Graphics &g, RectF rect, float max, float min, int sdi);
	MODE modes[100];
	uint32_t def_mode =  Z_STAB + XY_STAB;
	int modesI = 0;
	double p, step;
	double gdt = 0;
	double press_dt = 0;


/*
	float pitch, roll, sinPitch, sinRoll, cosPitch, cosRoll,yaw;
	float e_speedX, e_speedY;
	float e_accX, e_accY,_sinYaw,_cosYaw;
	float m7_accX, m7_accY;
	float f_pitch, f_roll;
	float w_accX, w_accY;*/
	void do_magic();
	
	int parser(byte buf[]);
public:
	int readLog();



	bool flags[ALL_ELEMENTS];
	Color color[ALL_ELEMENTS];
	std::wstring name[ALL_ELEMENTS];

	float bufkk0[128];
	int bufff0I = 0;

	Graph(char*);
	~Graph();
	int update(HDC hdc, RectF rect, double zoom, double pos);
	int updateGPS(HDC hdc, RectF rect, double zoom, double pos);
	int drawGPSmarkder(HDC hdc, RectF rect,  double pos);
	int drawText(HDC hdc, RectF rect, double pos);
};


