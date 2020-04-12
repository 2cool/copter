#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define gps Serial3



struct NAV_POSLLH {
	unsigned char cls;
	unsigned char id;
	unsigned short len;
	unsigned long iTOW;
	long lon;
	long lat;
	long height;
	long hMSL;
	unsigned long hAcc;
	unsigned long vAcc;
};

struct SEND_I2C {

	long lon;
	long lat;
	long height;
	uint8_t hAcc;
	uint8_t vAcc;
};

int gps_find_loop();
int gps_setup();
uint8_t processGPS();
SEND_I2C* gps_data();
