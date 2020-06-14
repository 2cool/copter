 
// 
// 
#include <cstdio>
#include <signal.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <stdio.h>
#include "commander.h"
#include "mi2c.h"
#include "Autopilot.h"
#include "Telemetry.h"
#include "Balance.h"
//#include "Hmc.h"
//#include "mpu.h"
#include "Prog.h"
#include "define.h"
#include "Hmc.h"
#include "Settings.h"
#include "Stabilization.h"
#include "debug.h"

#include "Log.h"

void CommanderClass::controls2zero() {
	yaw = yaw_offset = pitch = roll = throttle = 0;
}
bool CommanderClass::init()
{

	ppp_inet = true;
	telegram_bot = false;
	controls2zero();
	return true;
}

//static private final int s_COUNTER = 0;
//static private final int s_BUTTONS_S = 2;
//static private final int s_HEADING = 3;
//static private final int s_HEIGHT = 5;
//static private final int s_ACCELEROMETER = 7;



#define r_COUNTER  0
#define r_BUTTONS_S  2
#define r_HEADING  4
#define r_HEIGHT 6
#define r_ACCELEROMETER  8




#define  MAX_bit_on 219
#define  OFF_bit_on  36
#define  START_STOP 170












short old_height_c = 0;


/*
M,COUNTER,HEIGHT,HEADING,X,Y,
A,COUNTER,LAT,LON,HEIGHT,HEADING, SPEED,TIME,


*/
//max angel = 20 градусов
//




#define ANGK 0.1f
uint8_t data_errors = 0;







/*

B,COUNTER,MAX,   full throttle
B,COUNTER,OFF,   off throttle
B,COUNTER,S_S,		start stop


*/






bool CommanderClass::ButtonMessage(string msg){

#ifdef DEBUG_MODE
	printf("<- $s\n",msg.c_str());
#endif

	bool command_correct = false;


	if (msg.find("UP") == 0 && msg[2] >= '0' && msg[2] <= '9'){
		int_fast8_t n = (int_fast8_t)(msg[2] - '0');
		//Out.println(msg);
		//Out.println(n);
		/////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		Telemetry.getSettings(n);
		command_correct = true;
	}

	if (command_correct == false)
		cout << "WORNG MESSAGE\n";

	return command_correct;

}

float CommanderClass::getPitch(){ return  pitch; }
float CommanderClass::getRoll(){ return  roll; }



void CommanderClass::data_reset() {
	pitch = roll = 0;
	throttle = MIDDLE_POSITION;
}




int get32to8bMask(int v) {
	int mask = v & 255;
	mask ^= ((v >> 8) & 255);
	mask ^= ((v >> 16) & 255);
	mask ^= ((v >> 24) & 255);
	return mask;
}

int get16to8bMask(int v) {
	int mask = v & 255;
	mask ^= ((v >> 8) & 255);

	return mask;
}
float CommanderClass::get_yaw_minus_offset() {
	float y = yaw - yaw_offset;
	return wrap_180(y);
}
float CommanderClass::getYaw() {
	return yaw; 
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool CommanderClass::input(){


	if (shmPTR->connected==0 || shmPTR->commander_buf_len ==  0)
		return false;
	
	if (Autopilot.busy()) {
		shmPTR->commander_buf_len = 0;
		return true;
	}

	
	uint8_t *buf = shmPTR->commander_buf;
	if (shmPTR->commander_buf_len >= 12) {

		if (Log.writeTelemetry) { 
			Log.block_start(LOG::COMM,true);
			Log.loadMem(buf, shmPTR->commander_buf_len,false);
			Log.block_end(true);
		}



		uint32_t mode = *(uint32_t*)buf;
		int sec_mask = mode >> 24;
			
		mode &= 0x00ffffff;
		int mask = get32to8bMask(mode);

		int i = 4;
		int16_t i_throttle = *(int16_t*)(buf + i);
		mask ^= get16to8bMask(i_throttle);
		i += 2;
			

		int16_t i_yaw = *(int16_t*)(buf + i);
		i += 2;
		mask ^= get16to8bMask(i_yaw);
			

		int i_yaw_offset = *(int16_t*)(buf + i);
		i += 2;
		mask ^= get16to8bMask(i_yaw_offset);
			

		int i_pitch = *(int16_t*)(buf + i);
		i += 2;
		mask ^= get16to8bMask(i_pitch);
			
		int i_roll = *(int16_t*)(buf + i);
		i += 2;
		mask ^= get16to8bMask(i_roll);

#define CONTROL_MAX_ANGLE  45.0f
		if (mask == sec_mask) {


		/*	static int32_t old = 0;
			int32_t now = millis_();
			cout << now - old << endl; 
			old = now;
			*/
			Autopilot.last_time_data__recived = millis_(); 
			Autopilot.set_control_bits(mode);
			throttle = 0.00003125f*(float)i_throttle;
			yaw = -ANGK*(float)i_yaw;
			yaw_offset = ANGK*(float)i_yaw_offset;
			const float angle_k = Balance.get_max_angle() * (1.0f / CONTROL_MAX_ANGLE);
			pitch = angle_k * ANGK*(float)i_pitch;
			roll = angle_k * ANGK*(float)i_roll;

			if ((i + 3) < shmPTR->commander_buf_len) {
				string msg = "";
				msg += *(buf + i++);
				msg += *(buf + i++);
				msg += *(buf + i++);
				if (msg.find(m_PROGRAM) == 0 && Autopilot.progState()==false) {
					Prog.add(buf + i);
					mega_i2c.beep_code(B_COMMAND_RECEIVED);
				}
				else if (msg.find(m_SETTINGS) == 0) {
					Settings.load_(string((char*)(buf+i)),false);
					mega_i2c.beep_code(B_COMMAND_RECEIVED);
				}
				else if (msg.find(m_UPLOAD_SETTINGS) == 0) {
					Telemetry.getSettings(buf[i++]);
					//mega_i2c.beep_code(B_COMMAND_RECEIVED);
				}
				else if (msg.find(m_FPV) == 0) {
					static bool video_stream_off = true;
					//cout << "FPV\n";
					Autopilot.fpv_command_recived();
					video_stream_off ^= true;
					shmPTR->fpv_adr = *(buf + i++);			//if fpv_adr == 0 - video stream off
					shmPTR->fpv_port = *(int16_t*)(buf + i);
					i += 2;
					shmPTR->fpv_zoom = *(buf + i++);
					shmPTR->fpv_code = *(int16_t*)(buf + i); 
					i += 2;
				}
			}

			//Debug.dump( pitch, roll,yaw,yaw_offset);
		}
		else {
			cout << "COMMANDER ERROR\n";
		}
		shmPTR->commander_buf_len = 0;
		//cout << "Pitch=" << pitch << "; Roll=" << roll << "; Yaw=" << yaw<<endl;
		return true;
	}
	else
	{
		shmPTR->commander_buf_len = 0;
		cout << "<12\n";
		return true;
	}
	

	return false;
}






string CommanderClass::get_set() {
	string s = (ppp_inet) ? "1" : "0";
	s += ",";
	s+= (telegram_bot) ? "1" : "0";
	

	return s;
}

void CommanderClass::set(const float buf[]) {

	
	//shmPTR->fpv_adr = (shmPTR->client_addr&0xffffff00) | (uint8_t)buf[0];
	//shmPTR->fpv_port = buf[1];
	//shmPTR->fpv_zoom = buf[2];//0
	ppp_inet = buf[0] > 0;
	telegram_bot = buf[1] > 0;
	if (telegram_bot)
		ppp_inet = true;
	//cout << "trans adr %f\n", buf[0];
	//thread t(stop_stream);
//	t.detach();

}






CommanderClass Commander;

