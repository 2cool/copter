#pragma once
#include "GPS.h"
//enum {MPU_DDNE=0,MS5611_DONE=2,COMPASS_DONE=4, AUTOPILOT_DONE = 8,GPS_DONE=16,TELEMETRY_DONE=32,CONTROL_DONE=64,BASE_DONE = 128};
class LogReader
{

	struct Sensors_Data
	{
		int done;
		//mpu
		int64_t time;
		int16_t a[3];              // [x, y, z]            accel vector
		int16_t g[3];              // [x, y, z]            gyro vector


		//ms5611 barom

		byte i_readTemperature;
		float pressure;

		//hmc; compas

		uint8_t buffer[6];

		//base

		int16_t sh[6];
		float base[12];
		float   yaw_correction_angle;

		//gps
		SEND_I2C gps;

		//telemetry
		uint16_t data[5];

		//control



		//autopilot
		uint32_t control_bits;

	};



private:

	int readLog();

	void mpu_parser();
	void ms5611_parser();
	void gps_parser();
	void hmc_parser();
	void hmc_parser_base();
	void telemetry_parser();
	void commander_parser(const int);
	void set(const uint8_t);
	void unset(const uint8_t);
	bool test(const uint8_t);
public:
	Sensors_Data sd;
	LogReader();
	int parser(const uint8_t);
	
};
extern LogReader logR;
