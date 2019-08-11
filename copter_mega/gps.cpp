#include "gps.h"
#include <stdint.h>




const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };



NAV_POSLLH posllh;
SEND_I2C   s_i2c;

uint8_t counter = 0;

unsigned long oldITOW = 0;
SEND_I2C* gps_data() { return &s_i2c; }
static int fpos = 0;
static unsigned char checksum[2];
const int payloadSize = sizeof(NAV_POSLLH);
uint8_t processGPS() {
	unsigned char c = gps.read();
	if (fpos < 2) {
		if (c == UBX_HEADER[fpos]) {
			checksum[fpos] = 0;
			fpos++;
		}
		else
			fpos = 0;
	}
	else {
		if ((fpos - 2) < payloadSize) {
			((unsigned char*)(&posllh))[fpos - 2] = c;
			checksum[0] += c;
			checksum[1] += checksum[0];
		}
		fpos++;
		if (fpos == (payloadSize + 3)) {
			if (c != checksum[0]) {
				fpos = 0;
			}
		}
		else if (fpos == (payloadSize + 4)) {
			fpos = 0;
			if (c == checksum[1]) {
				s_i2c.lat = posllh.lat;
				s_i2c.lon = posllh.lon;
				s_i2c.height = posllh.height;
				posllh.vAcc >>= 10;
				posllh.hAcc >>= 10;
				s_i2c.vAcc =(uint8_t)((posllh.vAcc > 255)?255:posllh.vAcc);
				s_i2c.hAcc =(uint8_t)((posllh.hAcc > 255)?255:posllh.hAcc);

				/*
				if ((posllh.iTOW- oldITOW) > 110)
					Serial.println(posllh.iTOW - oldITOW);
				oldITOW = posllh.iTOW;
				*/
				return  128 | ((counter++)&127);
			}
		}
		else if (fpos > (payloadSize + 4)) {
			fpos = 0;
		}
	}
	return 0;
}















uint8_t GGA_d[] = { 181, 98, 6, 1, 3, 0, 240, 0, 0, 250, 15 };
uint8_t GLL_d[] = { 181, 98, 6, 1, 3, 0, 240, 1, 0, 251, 17 };
uint8_t GSA_d[] = { 181, 98, 6, 1, 3, 0, 240, 2, 0, 252, 19 };
uint8_t GSV_d[] = { 181, 98, 6, 1, 3, 0, 240, 3, 0, 253, 21 };
uint8_t RMC_d[] = { 181, 98, 6, 1, 3, 0, 240, 4, 0, 254, 23 };
uint8_t VTG_d[] = { 181, 98, 6, 1, 3, 0, 240, 5, 0, 255, 25 };
//uint8_t VELECEF[] = { 181, 98, 6, 1, 3, 0, 1, 17, 1, 29, 101 };
int8_t POSLLH[] = { 181, 98, 6, 1, 3, 0, 1, 2, 1, 14, 71 };
uint8_t ghz10[] = { 181, 98, 6, 8, 6, 0, 100, 0, 1, 0, 1, 0, 122, 18, 181, 98, 6, 8, 0, 0, 14, 48 };
uint8_t baud115200[] = { 181, 98, 6, 0, 20, 0, 1, 0, 0, 0, 208, 8, 0, 0, 0, 194, 1, 0, 35, 0, 35, 0, 0, 0, 0, 0, 252, 30, 181, 98, 6, 0, 1, 0, 1, 8, 34 };


int gps_setup() {
	gps.begin(9600);
	delay(100);
	//gps.print("$PUBX,41,1,0003,0001,115200,0*1E\r\n");
//	for (int i = 0; i < 37; i++)
	//	gps.write((uint8_t)baud115200[i]);
	//delay(100);
	//gps.end();
//	delay(200);
//	gps.begin(115200);
	//delay(100);

	//for (int i = 0; i < 11; i++)
	//	gps.write((uint8_t)VELECEF[i]);
	//delay(100);
	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)POSLLH[i]);
	delay(100);
	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)GGA_d[i]);
	delay(100);
	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)GLL_d[i]);
	delay(100);
	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)GSA_d[i]);
	delay(100);

	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)GSV_d[i]);
	delay(100);
	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)RMC_d[i]);
	delay(100);
	for (int i = 0; i < 11; i++)
		gps.write((uint8_t)VTG_d[i]);
	delay(100);

	delay(100);
	for (int i = 0; i < 22; i++)
		gps.write((uint8_t)ghz10[i]);
	return 0;
}


