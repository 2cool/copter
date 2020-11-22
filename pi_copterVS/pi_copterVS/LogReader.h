#pragma once
#include "GPS.h"
class LogReader
{

	struct Sensors_Data
	{
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
		int   yaw_correction_angle;

		//gps
		SEND_I2C gps;

		//telemetry
		uint16_t data[5];

		//control
		byte commander_buf[255];
		int commander_buf_len;
		//autopilot
		uint32_t control_bits;

	};



private:
	Sensors_Data sd;
	int readLog();
	bool initialize();
	void mpu_parser();
	void ms5611_parser();
	void gps_parser();
	void hmc_parser();
	void hmc_parser_base();
	void telemetry_parser();
	void commander_parser();
public:
	int parser(byte buf[]);
	void loop();
};
extern LogReader logR;
