#pragma once

#include <stdint.h>
#define SHMKEY "/home/igor/logs/"
#define TELEMETRY_BUF_SIZE 16384
enum { X, Y, Z };


//if fpv_recived this bit has inverted
enum {
	MOTORS_ON = 1, CONTROL_FALLING = 2, Z_STAB = 4, XY_STAB = 8, GO2HOME = 0x10, PROGRAM = 0x20, GIMBAL_AXIS = 0x40, FPV_RECIVED = 0x80,
	MPU_ACC_CALIBR = 0x100, MPU_GYRO_CALIBR = 0x200, COMPASS_CALIBR = 0x400, COMPASS_MOTOR_CALIBR = 0x800, SHUTDOWN = 0x1000, GIMBAL_PLUS = 0x2000,
	GIMBAL_MINUS = 0x4000, REBOOT = 0x8000, PROGRAM_LOADED= 0x10000, M_ON_AND_GO2HOME = 0x20000, M_ON_AND_PROG_START = 0x40000,WIFI_CAMERA_FOUND = 0x80000,INET_OK = 0x100000
};

/*
struct Stat_Fields {
	unsigned short wifi_Str : 8;    // 0..7   (3 bits)
	unsigned short nMonthDay : 6;    // 0..31  (6 bits)
	unsigned short nMonth : 5;    // 0..12  (5 bits)
	unsigned short nYear : 8;    // 0..100 (8 bits)
};
*/
struct Memory {

	bool internet_run,fpv_run,wifi_run;
	bool in_fly;
	uint8_t main_cnt, internet_cnt, wifi_cnt,fpv_cnt;


	uint32_t status;  //0-6 wifi signal,7-14-wifi_noise, 15-fpv_status, 17-inet status
	uint32_t control_bits;
	uint32_t control_bits_4_do;

	float pitch, roll, yaw;
	uint8_t accuracy_hor_pos_, accuracy_ver_pos_;
	long lat_, lon_, lat_home, lon_home;
	float  speedX, speedY, speedZ, dist2home;
	int32_t gps_altitude_, altitude_;
	int32_t pressure;
	float voltage, voltage_at_start;
	float m_current[4];


	uint8_t sms_at_work;
	bool inet_ok, ppp_run, sim800_reset,telegram_run, loger_run, stop_ppp_read_sms_start_ppp;
	int32_t sim800_reset_time;




	float fly_at_start, lowest_altitude_to_fly;

	
	bool run_main;
	uint8_t reboot;










	int commander_buf_len, telemetry_buf_len;
	uint32_t connected;
	uint32_t client_addr;
	uint8_t commander_buf[TELEMETRY_BUF_SIZE];
	uint8_t telemetry_buf[TELEMETRY_BUF_SIZE];

	uint8_t fpv_adr;
	uint16_t fpv_port;
	uint8_t fpv_zoom; //if 0 then fpv off;
	uint16_t fpv_code; // code for control xiaomi
};
