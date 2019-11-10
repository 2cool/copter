
#ifndef DEFINE_H
#define DEFINE_H

#include "glob_header.h" 
#include "WProgram.h"

//#define OFF_MOTOR_IF_LOST_CONNECTION  
//#define OFF_TIMELAG
//-----------------------------------------------------------------
#define FLY_EMULATOR
//----------------------------------------------------------------
//#define YAW_OFF
//#define XY_SAFE_AREA 200
//#define Z_SAFE_AREA 40

#define MAX_ACC 7
//#define DEBUG_MODE
#define GPS_ALT_MAX_ERROR 30
#define MAX_BAROMETR_ERRORS 40


#define WORK_WITH_WIFI
#define LED_ON
#define LOST_BEEP
#define pwm_MAX_THROTTLE 32000
#define pwm_OFF_THROTTLE 16000
#define  ARDUINO_ADDR 9

//#define FASLE_GPS_STILL
//#define NO_BATTERY

#ifdef FLY_EMULATOR
//#define FASLE_GPS_STILL
//#define MOTORS_OFF
#define COMPAS_MOTORS_OFF  
#define FALSE_ALTITUDE 30
#else
#define BUZZER_R
//#define GYRO_CALIBR
//#define ON_MAX_G_MOTORS_OFF
#endif

//#define MAX_HIGHT_ERROR_TO_FALL 100
#define MAX_DIST_ERROR_TO_FALL 200
#define NO_GPS__DATA 10e3

#define MIN_ACUR_HOR_POS_2_START_ 5
#define MIN_ACUR_HOR_POS_4_JAMM 30


#define MAX_ANGLE 35
#define MIN_ANGLE 15
#define COS_MIN_ANGLE 0.966f


#define DOWN_IF_HIGHER_THEN_ON_FLY_TO_HOME 200
#define HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME 80
#define FAST_DESENDING_TO_HIGH 15

#define MIN_STAB_HOR_SPEED 0.2
#define MAX_HOR_SPEED 10
#define MAX_VER_SPEED_PLUS 5
#define MAX_VER_SPEED_MINUS -5

#define ACCURACY_XY 5
#define ACCURACY_Z 3
#define HOWER_TIME 30

#define MAX_DELTA 0.2f
#define MAX_YAW_DELTA 0.15f
#define FULL_THROTTLE_ 1.0f
#define MAX_THROTTLE (FULL_THROTTLE_-MAX_DELTA)
#define OVER_THROTTLE 0.9f


#define CONNECTION_LOST__TIMEOUT 5e3
#define TIMEOUT__LAG 500

#define MIDDLE_POSITION 0.5f
#define HOVER_THROTHLE 0.5f
#define MIN_THROTTLE 0.4f
#define FALLING_THROTTLE 0.45f

#define STOP_THROTTLE_ 0.2f


#define PRESSURE_AT_0 101325
#define MAX_G 32760  
#define RAD2GRAD 57.295779513082320876798154814105
#define GRAD2RAD 0.01745329251994329576923690768489
#define GRAVITY_G 9.81

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_PI(x) (x < -PI ? x+TWO_PI : (x > PI ? x - TWO_PI: x))

#define HALL_EFFECT_SENSOR_MAX_CURRENT 20
#define e_HALL_ERROR "HRR"
#define e_ARDUINO_RW_ERROR "ARW"
#define e_TOO_STRONG_WIND "TSW"
//#define e_TOO_HIGHT_FROM_NEED "HFN"
#define e_COMMPAS_RW_ERROR "CRW"
#define e_BAROMETR_RW_ERROR "BRW"
#define e_BAROMETR_ERROR "BER"
#define SETTINGS_ARRAY_SIZE 10
#define SETTINGS_IS_OK 1
#define SETTINGS_ERROR 0
#define BEGIN_CONVERSATION "GET"
#define e_OUT_OF_PER_H   "TFR"
#define e_OUT_OF_PER_V   "THG"
#define e_NO_WIFI_2_LONG "NWF"
#define e_NO_INTERNET	"WWW"
#define e_LOW_VOLTAGE    "LWV"
#define e_GPS_ERROR      "GPE"
#define e_NO_GPS_2_LONG  "NGP"
#define e_MAX_ACCELERATION    "MXG"
#define e_GPS_ERRORS_M_50 "GER"
#define e_GPS_NO_UPDATE  "GRR"
#define e_VOLTAGE_ERROR "TEE"
#define e_VOLT_MON_ERROR "VME"
#define e_PRESURE_DEV_ZER "PRE"
#define e_BATERY_OFF_GO_2_HOME "BOH"
#define e_CONNECTION_TIME_LAG "LAG"
#define e_LOST_CONNECTION "LST"
#define e_SYSTEM_MALFUNCTION "WDT"
#define e_ESK_ERROR			"ESK"
#define e_CALIBRATING		"CLB"
#define e_BARROMETR_FAULT   "BFT"
#define e_BARROMETR_FAULT_COPTER_AT_HOME "BCH"
#define e_PROG_INDEX_ERROR "PIE"
#define e_PROG_TOO_LONG_FROM_START "TOO"
#define e_PROG_TOO_LONG_DISTANCE "TOL"
#define e_port_R_W_ERROR "PRW"
#define e_MPU_TOO_LONG "MTL"
#define e_MPU_RW_ERROR "MRW"
#define i_OFF_MOTORS     "MD0"
#define i_CONTROL_FALL   "CNF"
#define i_MAX_THR        "MXT"


#define m_START_STOP  "S_S"
#define m_HOLD_HIGHT  "AHD"
#define m_SMART_CNTR  "SCT"
#define m_GO_2_HOME   "THM"
#define m_MPU_GYRO_CAL "STS"
#define m_COMPAS_CAL  "CMC"
#define m_MAX_THR     "MAX"
#define m_OFF_THR     "OFF"
#define m_MPU_NEW_CAL "HOR"

#define m_GIMBAL_PA   "CDN"
#define m_GIMBAL_PS   "CUP"
#define m_DIRECTION_C "CMP"
#define m_XY_CONTROL  "HRT"
#define m_MOTOR_COMP_C "MCC"
#define m_START_PROG   "SRP"
#define m_SETTINGS	   "SET"
#define m_UPLOAD_SETTINGS "UPS"
#define m_PROGRAM	   "PRG"
#define m_FPV			"FPV"
enum { B_CONNECTION_LOST = 1, B_MS611_ERROR, B_ACC_ERROR, B_LOW_VOLTAGE, B_GPS_ACCURACY_E, B_MPU_TOO_LONG, B_GPS_TOO_LONG , B_BARROMETR_ERR,B_I2C_ERR, B_COMMAND_RECEIVED};



struct SEND_I2C {

	int32_t lon;
	int32_t lat;
	int32_t height;
	uint8_t hAcc;
	uint8_t vAcc;
};




extern struct Memory *shmPTR;


#endif
