
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include "Telemetry.h"
#include "debug.h"
#include "define.h"
#include "mi2c.h"
#include "mpu.h"




int fd;
/*
int Megai2c::getsim(char * str) {
	char reg = 3;
	char sim_count;
	write(fd, &reg, 1);
	int ret = read(fd, &sim_count, 1);
	int res = 0;
	if (ret == 1 && sim_count > 0) {
		res = read(fd, str, sim_count);
		//for (int i = 0; i < sim_count; i++)
		//	printf("%c", str[i+ shift]);
	}
	return res;
}
*/



void Megai2c::settings(uint16_t overloadTime, uint16_t overloadVal, uint8_t overloadCnt) {
	char send_buf[6];
	send_buf[0] = 2;
	*((uint16_t*)&send_buf[1]) = overloadTime;
	*((uint16_t*)&send_buf[3]) = overloadVal;
	send_buf[5] = overloadCnt;
	write(fd, send_buf, 6);
}
/*
int Megai2c::send2sim(const char *str, int len) {
	gsm_send_buf[0] = 2;
	memcpy(gsm_send_buf + 1, str, len);
	//usleep(50);
	write(fd, gsm_send_buf, len + 1);
	//	for (int i = 1; i<len+1; i++)
	//	printf("%c", gsm_send_buf[i]);
}

	
	+CMTI: "SM",1
	+CMTI: "SM",2
	...

	+CMTI: "ME",21
	приходят смски

	RING

	NO CARRIER
	*/
void Megai2c::m_parser(char *str, int len) {
const static char no_carrier[] = "NO CARRIER";
const static char ring[] = "RING";
const static char sms[] = "+CMTI: \"SM\",";
static int smsi = 0, ringi=0, no_carrieri=0;
static int smsN = 0;
static int sms_received = 0;
	for (int i = 0; i < len; i++) {


		if (sms_received == -1) {
			if (str[i] >= '0' && str[i] <= '9') {
				smsN = smsN * 10 + str[i] - '0';
			}
			else {
				sms_received = smsN;
				cout << "SMS " << sms_received << "\t"<<Mpu.timed << endl;
				smsN ^= smsN;

			//	sim.readSMS(sms_received,  true, true);
			}
		}
		else {
			if (str[i] == sms[smsi++]) {
				if (smsi == sizeof(sms) - 1) {
					sms_received = -1;
					smsi ^= smsi;
				}
			}
			else
				smsi ^= smsi;

			if (ring_received == false) {
				if (str[i] == ring[ringi++]) {
					if (ringi == sizeof(ring) - 1) {
						ring_received = true;
						cout << "RING" << "\t"<<Mpu.timed << endl;
						ringi ^= ringi;
					}
				}
				else
					ringi ^= ringi;
			}
			else {
				if (str[i] == no_carrier[no_carrieri++]) {
					if (no_carrieri == sizeof(no_carrier) - 1) {
						ring_received = false;
						cout << "NO CARRIER" << "\t"<<Mpu.timed << endl;
						no_carrieri ^= no_carrieri;
					}
				}
				else
					no_carrieri ^= no_carrieri;
			}
		}
	}
}

int Megai2c::init()
{
	if (init_shmPTR())
		return 0;
	current_led_mode = 100;
	ring_received = false;
	if ((fd = open("/dev/i2c-0", O_RDWR)) < 0) {
		cout << "Failed to open /dev/i2c-0" << "\t"<<Mpu.timed << endl;
		return -1;
	}
	if (ioctl(fd, I2C_SLAVE, ARDUINO_ADDR) < 0) {
		cout << "Failed to acquire /dev/i2c-0 access and/or talk to slave." << "\t"<<Mpu.timed << endl;
		return -1;
	}
	//--------------------------------init sound & colors 
	char buf[7];

	buf[0] = 7 + (DO_SOUND << 3);
	// no connection RGB
	buf[1] = 1;
	buf[2] = 0;
	buf[3] = 0;
	// ring RGB
	buf[4] = 255;
	buf[5] = 0;
	buf[6] = 0;
	write(fd, buf, 7);
	shmPTR->sim800_reset_time = 0;

	//mega_i2c.settings(300, 10, 5); упал 2019 07 10 при збое и резком  рывке
	mega_i2c.settings(300, 10, 15);   //if not execute/ copters motors not start;
	return 0;
}

void Megai2c::beep_code(uint8_t c) {
	cout << "BEEP:" << (int)c << endl;
	if (DO_SOUND) {
		char chBuf[] = { 1,c };
		write(fd, chBuf, 2);
	}
}

void Megai2c::Buzzer(const bool on) {

}


uint16_t Megai2c::correct(const float n) {    //0-это

	return (uint16_t)(pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE));

}

void Megai2c::throttle(const float n[]) {
#ifdef FLY_EMULATOR
	Emu.update(n, Mpu.dt);
#else
	uint16_t pwm_out[5];
	pwm_out[0] = 0;
	pwm_out[1] = (uint16_t)(pwm_OFF_THROTTLE + n[0] * pwm_OFF_THROTTLE);
	pwm_out[2] = (uint16_t)(pwm_OFF_THROTTLE + n[1] * pwm_OFF_THROTTLE);
	pwm_out[3] = (uint16_t)(pwm_OFF_THROTTLE + n[2] * pwm_OFF_THROTTLE);
	pwm_out[4] = (uint16_t)(pwm_OFF_THROTTLE + n[3] * pwm_OFF_THROTTLE);
	char* chBuf = (char*)pwm_out;
	if (write(fd, chBuf + 1, 9) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write power error" <<  Mpu.timed  << endl;
		mega_i2c.beep_code(B_I2C_ERR);
	}
#endif
}

void Megai2c::set_led_color(uint8_t n, uint8_t r, uint8_t g, uint8_t b) {
	char buf[4];
	buf[0] = 3 + (n << 3);
	buf[1] = *(char*)&r;
	buf[2] = *(char*)&g;
	buf[3] = *(char*)&b;
	if (write(fd, buf, 4) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write LED error" << Mpu.timed << endl;
		mega_i2c.beep_code(B_I2C_ERR);
	}
}
void Megai2c::sim800_reset() {
	char chBuf[] = { 1,16 };
	shmPTR->sim800_reset_time = millis();
	if (write(fd, chBuf, 2) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write sim800 error" << Mpu.timed << endl;
		mega_i2c.beep_code(B_I2C_ERR);
	}
}
//0.35555555555555555555555555555556 = 1град
bool Megai2c::gimagl(float pitch, float roll) {  // добавить поворот вмесете с коптером пра опред обст
	if (pitch <= 80 && pitch >= -45) { 

		//Serial.printf("camAng="); Serial.println(angle);
		pitch = pwm_OFF_THROTTLE + (180 - pitch)*44.444444;
		roll = pwm_OFF_THROTTLE + (180 + roll)*44.4444;
		char buf[6];

		buf[1] = 8;
		((uint16_t*)buf)[1] = (uint16_t)(pitch);
		((uint16_t*)buf)[2] = (uint16_t)roll;
		if (write(fd, buf + 1, 5) == -1) {
			Telemetry.addMessage(e_ARDUINO_RW_ERROR);
			cout << "arduino write gimbal error" << Mpu.timed << endl;
			mega_i2c.beep_code(B_I2C_ERR);
		}
		return true;
	}
	else
		return false;
}


int Megai2c::get_gps(SEND_I2C *gps_d) {

	char reg = 1;
	char bit_field;
	if (write(fd, &reg, 1) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write get_gps error" << Mpu.timed << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		return -1;
	}
	int res = read(fd, &bit_field, 1);
	if (res == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino read get_gps error" << Mpu.timed << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		return -1;
	}


	static double last_ring_time = 0;

	if (last_ring_time > 0 && last_ring_time + 10 < Mpu.timed) {
		last_ring_time = 0;
		shmPTR->stop_ppp_read_sms_start_ppp = true;
		cout << "RING_BIT sended..." << "\t"<<Mpu.timed << endl;
		
	}


	if (bit_field & 1 && shmPTR->sim800_reset_time == 0) {
		if (last_ring_time==0)
			cout << "RING_BIT" << "\t"<<Mpu.timed << endl;//при заходе смс при ppp
		last_ring_time = Mpu.timed;
		
		///stop servises, stop ppp? read sms and do. start ppp and services again
		
	}

	if (bit_field & 2) {
		res = read(fd, (char*)gps_d, sizeof(SEND_I2C));
		if (res == -1) {
			Telemetry.addMessage(e_ARDUINO_RW_ERROR);
			cout << "arduino write bit_field & 2 error" << Mpu.timed << endl;
			mega_i2c.beep_code(B_I2C_ERR);
		}
		return res;
	}
	else {
		return 0;
	}

}

int Megai2c::getiiiiv(char *iiiiv) {
	char reg = 0;
	if (write(fd, &reg, 1) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write iiiiv error" << Mpu.timed << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		return -1;
	}
	
	if (read(fd, (char*)iiiiv, 10) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino read iiiiv error" << Mpu.timed << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		return -1;
	}
	return 0;
}


#define LED_MODS 3
//против часовой стрелки с 3 ноги.

#define RED 1,0,0
#define GRN 0,1,0
#define BLE 0,0,1
#define BLK 0,0,0
#define YEL 1,1,0

const uint8_t collors[][8][3] = { 
	{ { GRN },{ GRN },{ GRN },{ GRN },{ GRN },{ GRN },{ GRN },{ GRN } } ,// green
	{ { RED },{ RED },{ GRN },{ GRN },{ GRN },{ GRN },{ RED },{ RED } } ,// green red
	{ { RED },{ RED },{ RED },{ RED },{ RED },{ RED },{ RED },{ RED } }, // red
	{ { RED },{ RED },{ BLE },{ BLE },{ BLE },{ BLE },{ RED },{ RED } }, // blu red
	{ { RED },{ RED },{ YEL },{ YEL },{ YEL },{ YEL },{ RED },{ RED } }, // yel red
	{ { BLK },{ BLK },{ BLK },{ BLK },{ BLK },{ BLK },{ BLK },{ BLK } } // black
};
	
void Megai2c::set_led_mode(uint8_t n, uint8_t briht, bool pulse) {
	static uint32_t old_led_mode = 0;
	static uint8_t pulse_f0 = 1;
	static int cur_led_n = 8;
	static double last_timed = 0;
	static uint8_t pulse_f = 1;

	if (cur_led_n < 8) {
		uint8_t b = briht * pulse_f;
		set_led_color(cur_led_n +1, collors[n][cur_led_n][0]*b, collors[n][cur_led_n][1] * b, collors[n][cur_led_n][2] * b);
		cur_led_n++;
	}
	else {
		if (pulse)
			pulse_f0 ^= 1;
		else
			pulse_f0 = 1;
		uint32_t t = n | (briht <<8) | (pulse_f0 <<16);
		if (t != old_led_mode){
			old_led_mode = t;
			//cout << "LED "<<t << endl;
			float dt = Mpu.timed - last_timed;
			if (current_led_mode != n || dt > 0.1) {
				if (pulse)
					pulse_f ^= 1;
				else
					pulse_f = 1;
				last_timed = Mpu.timed;
				cur_led_n = 0;
				current_led_mode = n;
			}
		}
		
		
		
		







	}
}
void Megai2c::sound(const float f) {
#ifdef BUZZER_R
	//OCR4B = 32000-(int16_t)f; 
#endif

}

Megai2c mega_i2c;

