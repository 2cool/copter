#include "stdafx.h"
#include "commander.h"

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
#define ANGK 0.1f
void commander::parser(byte buf[], int i, int len, int cont_bits, bool filter, bool rotate) {

	i += 2;
	uint32_t mode = *(uint32_t*)(buf+i);
	int sec_mask = mode >> 24;

	mode &= 0x00ffffff;
	int mask = get32to8bMask(mode);

	i += 4;
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



	if (mask == sec_mask) {
		//Autopilot.last_time_data__recived = millis_();
		//Autopilot.set_control_bits(mode);
		throttle = 0.00003125f * (float)i_throttle;
		yaw = -ANGK * (float)i_yaw;
		yaw_offset = ANGK * (float)i_yaw_offset;
		pitch = ANGK * (float)i_pitch;
		roll = ANGK * (float)i_roll;
		/*
		if ((i + 3) < len) {
			string msg = "";
			msg += *(buf + i++);
			msg += *(buf + i++);
			msg += *(buf + i++);
			if (msg.find(m_PROGRAM) == 0 && Autopilot.progState() == false) {
				Prog.add(buf + i);
				mega_i2c.beep_code(B_COMMAND_RECEIVED);
			}
			else if (msg.find(m_SETTINGS) == 0) {
				Settings.load_(string((char*)(buf + i)), false);
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
		}*/
	}
	else {
		
	}

}
commander com;