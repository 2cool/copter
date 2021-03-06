#include <bitset>
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

#define MAX90_THROTTLE   45604
#define pwm_MIN_THROTTLE 26402
#define pwm_MAX_THROTTLE 48004
#define pwm_OFF_THROTTLE 24002
#define OVER_CURRENT 19
#define ESC_CALIBR 0


int open_i2c_() {
	int fd4S;
	if ((fd4S = open("/dev/i2c-0", O_RDWR)) < 0) {
		cout << "Failed to open /dev/i2c-0" << "\t" << millis_() << endl;
		return -1;
	}
	if (ioctl(fd4S, I2C_SLAVE, ARDUINO_ADDR) < 0) {
		cout << "Failed to acquire /dev/i2c-0 access and/or talk to slave." << "\t" << millis_() << endl;
		return -1;
	}
	return fd4S;

}



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


//add flag for next poweron start with full throthle
void Megai2c::settings(float overCurrentVal, uint8_t esc_calibr0, uint8_t esc_calibr1, uint8_t esc_calibr2, uint8_t esc_calibr3) {
	char send_buf[7];
	send_buf[0] = 2;

	if (overCurrentVal > HALL_EFFECT_SENSOR_MAX_CURRENT)
		overCurrentVal = HALL_EFFECT_SENSOR_MAX_CURRENT;
	uint16_t overloadVal = (uint16_t)1024 - ((overCurrentVal / HALL_EFFECT_SENSOR_MAX_CURRENT) * 1024);
	*((uint16_t*)&send_buf[1]) = overloadVal;
	send_buf[3] = esc_calibr0;
	send_buf[4] = esc_calibr1;
	send_buf[5] = esc_calibr2;
	send_buf[6] = esc_calibr3;

	const int fd = open_i2c_();

	if (fd==-1 || write(fd, send_buf, 7) == -1){

		cout << "read set reg Failed to write to the arduino i2c bus ." << "\t" << millis_() << endl;
	}
	close(fd);

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
	�������� �����

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
				cout << "SMS " << sms_received << "\t"<<millis_() << endl;
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
						cout << "RING" << "\t"<< millis_() << endl;
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
						cout << "NO CARRIER" << "\t"<< millis_() << endl;
						no_carrieri ^= no_carrieri;
					}
				}
				else
					no_carrieri ^= no_carrieri;
			}
		}
	}
}
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ init @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
int Megai2c::init()
{
	if (init_shmPTR())
		return 0;
#ifndef FLY_EMULATOR
#ifndef LOG_READER
	current_led_mode = 100;
	ring_received = false;
	const int fd = open_i2c_();
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
	if (fd==-1 || write(fd, buf, 7)==-1)
		cout<< "write set reg Failed to write to the arduino i2c bus ." << "\t" << millis_() << endl;
	close(fd);
	shmPTR->sim800_reset_time = 0;

	mega_i2c.settings(OVER_CURRENT, ESC_CALIBR, ESC_CALIBR, ESC_CALIBR, ESC_CALIBR);
#endif
#endif
	return 0;
}

void Megai2c::beep_code(uint8_t c) {
	static uint8_t old_code = 255;
	static int32_t old_beep_time = 0;
	const int32_t _ct = millis_();
	if (c != old_code || (_ct - old_beep_time) >= 500) {
		old_beep_time = _ct;
		old_code = c;
		//cout << "BEEP:" << (int)c << endl;
		if (DO_SOUND) {
			char chBuf[] = { 1,c };
			const int fd = open_i2c_();
			if (fd==-1 || write(fd, chBuf, 2)==-1)
				cout << "write  reg Failed to write to the arduino i2c bus ." << "\t" << millis_() << endl;
			close(fd);
		}
	}
}

void Megai2c::Buzzer(const bool on) {

}


uint16_t Megai2c::correct(const float n) {    //0-���

	return (uint16_t)(pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE));

}

void Megai2c::throttle(const float _n[]) {
	static bool fn[4] = { false,false,false,false };
	float n[4] = { _n[0],_n[1],_n[2],_n[3] };
	static float down[4];

	for (int i = 0; i < 4; i++) {
		if (fn[i] == false) {
			if (n[i] >= 0.05) {
				fn[i] = true;
				down[i] = 0;
			}
		}else 
			if (n[i] == 0) {
				down[i] -= 0.006;
				if (down[i] < 0) {
					down[i] = 0;
					fn[i] = false;
				}
				n[i] = down[i];
			}else
				down[i] = n[i];
	}


#ifndef LOG_READER
#ifdef FLY_EMULATOR
	Emu.update(n, Mpu.get_dt());
#else
	uint16_t pwm_out[5];
	pwm_out[0] = 0;
	pwm_out[1] = (uint16_t)(pwm_OFF_THROTTLE + n[0] * pwm_OFF_THROTTLE);
	pwm_out[2] = (uint16_t)(pwm_OFF_THROTTLE + n[1] * pwm_OFF_THROTTLE);
	pwm_out[3] = (uint16_t)(pwm_OFF_THROTTLE + n[2] * pwm_OFF_THROTTLE);
	pwm_out[4] = (uint16_t)(pwm_OFF_THROTTLE + n[3] * pwm_OFF_THROTTLE);
	char* chBuf = (char*)pwm_out;
	const int fd = open_i2c_();
	if (fd == -1 || write(fd, chBuf + 1, 9) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write power error " << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
	}
	close(fd);
#endif
#endif
}

void Megai2c::set_led_color(uint8_t n, uint8_t r, uint8_t g, uint8_t b) {
	/*
	char buf[4];
	buf[0] = 3 + (n << 3);
	buf[1] = *(char*)&r;
	buf[2] = *(char*)&g;
	buf[3] = *(char*)&b;
	const int fd = open_i2c_();
	if (fd == -1 || write(fd, buf, 4) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write LED error" << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
	}
	close(fd);
	*/
}
void Megai2c::sim800_reset() {
	/*
	char chBuf[] = { 1,16 };
	shmPTR->sim800_reset_time = millis_();
	const int fd = open_i2c_();
	if (fd == -1 || write(fd, chBuf, 2) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write sim800 error" << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
	}
	close(fd);
	*/
}
//0.35555555555555555555555555555556 = 1����
bool Megai2c::gimagl(float pitch, float roll) {  // �������� ������� ������� � �������� ��� ����� ����
#ifndef FLY_EMULATOR
#ifndef LOG_READER
	if (pitch <= 80 && pitch >= -45 && roll > -75 && roll < 110) { 

		//Serial.printf("camAng="); Serial.println(angle);
		pitch = pwm_OFF_THROTTLE + (180 - pitch) * 66.67;
		roll = pwm_OFF_THROTTLE + (180 + roll) * 66.67;
		char buf[6];

		buf[1] = 8;
		((uint16_t*)buf)[2] = (uint16_t)roll;
		((uint16_t*)buf)[1] = (uint16_t)pitch;
		const int fd = open_i2c_();
		if (fd == -1 || write(fd, buf + 1, 5) == -1) {
			Telemetry.addMessage(e_ARDUINO_RW_ERROR);
			cout << "arduino write gimbal error" << millis_() << endl;
			mega_i2c.beep_code(B_I2C_ERR);
		}
		close(fd);
		return true;
	}
	else
#endif
#endif
		return false;
}

enum {RING=1, GPS_READY=2, OVERLOAD= 0b00111100};

int Megai2c::get_gps(SEND_I2C *gps_d) { 

	char reg = 1;
	char bit_field;
	const int fd = open_i2c_();
	if (write(fd, &reg, 1) == -1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write get_gps error" << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		close(fd);
		return -1;
	}
	int res = read(fd, &bit_field, 1);
	if (res != 1) {
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino read get_gps error" << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		close(fd);
		return -1;
	}

	//--------------------------------------------------------------------------

	#ifdef START_INET
	static int32_t last_ring_time = 0;
	if (last_ring_time > 0 && last_ring_time + 10e3 < millis_()) {
		last_ring_time = 0;
		shmPTR->stop_ppp_read_sms_start_ppp = true;
		cout << "RING_BIT sended..." << "\t"<< millis_() << endl;
	}
	if (bit_field & RING && shmPTR->sim800_reset_time == 0) {
		if (last_ring_time==0)
			cout << "RING_BIT" << "\t"<< millis_() << endl;//��� ������ ��� ��� ppp
		last_ring_time = millis_();
		///stop servises, stop ppp? read sms and do. start ppp and services again
	}
#endif
	//--------------------------------------------------------------------------
	motors_overload = (bit_field & OVERLOAD);
	if (motors_overload) {
		
		cout <<"OVERLOAD "<< std::bitset<4>(motors_overload>>2) <<"\t" << millis_()<<endl;
	}
	//--------------------------------------------------------------------------
	int ret = 0;
	if (bit_field & GPS_READY) {
		ret = read(fd, (char*)gps_d, sizeof(SEND_I2C));	
		if (ret==-1)
			cout << "arduino read gps_ error" << millis_() << endl;
	}
	close(fd);
	return ret;
	

}

int Megai2c::getiiiiv(char *iiiiv) {
	char reg = 0;
	const int fd = open_i2c_();
	if (fd == -1 || write(fd, &reg, 1) == -1) {
		close(fd);
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino write iiiiv error" << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		return -1;
	}
	
	if (read(fd, (char*)iiiiv, 10) == -1) {
		close(fd);
		Telemetry.addMessage(e_ARDUINO_RW_ERROR);
		cout << "arduino read iiiiv error" << millis_() << endl;
		mega_i2c.beep_code(B_I2C_ERR);
		return -1;
	}
	close(fd);
	return 0;
}


#define LED_MODS 3
//������ ������� ������� � 3 ����.

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
	{ { BLK },{ BLK },{ BLK },{ BLK },{ BLK },{ BLK },{ BLK },{ BLK } }, // black

	{ { RED },{ RED },{ RED },{ RED },{ RED },{ RED },{ RED },{ RED } }, // red
	{ { GRN },{ GRN },{ GRN },{ GRN },{ GRN },{ GRN },{ GRN },{ GRN } }, // green
	{ { BLE },{ BLE },{ BLE },{ BLE },{ BLE },{ BLE },{ BLE },{ BLE } } // blue


};
	
void Megai2c::set_led_mode(uint8_t n, uint8_t briht, bool pulse) {

	static uint32_t old_led_mode = 0;
	static uint8_t pulse_f0 = 1;
	static int cur_led_n = 8;
	static int32_t last_timed = 0;
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
			const int32_t _ct = millis_();
			const int32_t dt = _ct - last_timed;
			if (current_led_mode != n || dt > 100) {
				if (pulse)
					pulse_f ^= 1;
				else
					pulse_f = 1;
				last_timed = _ct;
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

