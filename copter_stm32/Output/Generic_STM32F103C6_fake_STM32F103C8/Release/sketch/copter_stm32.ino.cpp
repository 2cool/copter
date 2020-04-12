#line 1 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
#line 1 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"

#include <stdint.h>
#include <Arduino.h>
#include <Wire_slave.h>
#include <EEPROM.h>
#include "gps.h"

//#define PIN           27
//#define NUMPIXELS      8


//#define _16MHZ CS10
//#define _8MHZ CS11

//490 hz
//490 hz
//#define pwm_COUNTER		 65535
//333.33(3)hz
//#define pwm_COUNTER		 48000
#define MAX90_THROTTLE   61013
#define pwm_MIN_THROTTLE 35323
#define pwm_MAX_THROTTLE 64224
#define pwm_OFF_THROTTLE 32112

//#define SIM800 Serial2
//#define BUZZER 30
//#define RING 31
//#define SIM800_RESET 25

enum
{
	M0 = PA2, M1 = PA6, M2 = PA7, M3 = PA8, G_PITCH = PB0, G_ROLL = PB1, AM0 = PA0, AM1 = PA1, AM2 = PA3, AM3 = PA4, BAT = PA5
}
;

volatile uint8_t beep_code = 0;

uint8_t beeps_coder[] = { 0, B00001000, B00001001, B00001010, B00001011, B00001100, B00001101, B00001110, B00001111, B00000001, B00000010, B00000011, B00000100, B00000101, B00000110, B00000111 };  //4 beeps if 0 short 1 long beep

enum { ESC_CALIBR_ADR0, ESC_CALIBR_ADR1, ESC_CALIBR_ADR2, ESC_CALIBR_ADR3 }
;

#define b10 {100,0,0}
#define b01 {0,100,0}
#define b00 {0,0,0}
#define b11 {100,100,0}
uint8_t beeps_led[][2][3]{ {b00, b00}, { b10, b00},{b10, b01},{ b10, b01 },{b10, b11},{b11, b00},{b11, b01},{b11, b10},{b11, b11},{b00, b01,},{b00, b11},{b01, b00},{b01, b01},{b01, b10},{b01, b11} }
;

volatile bool ring = false, ring_to_send = false;

volatile uint16_t overloadVal = 0;

volatile bool gps_status = false;
volatile uint8_t pi_copter_color[8][3] = { { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 } };

enum { eNO_CON, eRING };
volatile uint8_t col[][3] = { { 1, 0, 0 }, { 255, 0, 0 } };
volatile bool  do_sound = true;



volatile uint8_t overloadF = 0;
volatile uint8_t new_colors_i = 0;
bool ring_was = false;

#define PAUSE_TIME 100
#define LONG_BEEP 600
#define SHORT_BEEP 200




#ifdef PRINT
#define printf(args) Serial.println (args);
#else
#define printf(args) 
#endif

uint8_t beep_bit_n = 0;
uint32_t beep_time = 0;
uint8_t old_beep_code = 255;
#line 83 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void beep();
#line 130 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void stop_motors();
#line 139 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_0(const float n);
#line 142 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_1(const float n);
#line 146 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_2(const float n);
#line 149 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_3(const float n);
#line 170 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void receiveEvent(int countToRead);
#line 280 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void requestEvent();
#line 332 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void setup();
#line 426 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void loop();
#line 83 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void beep() {

	if (old_beep_code != beep_code)
	{
		old_beep_code = beep_code;
		for (int i = 0; i < 8; i += 2)
		{
			//pixels.setPixelColor(i, pixels.Color(beeps_led[beep_code][0][0], beeps_led[beep_code][0][1], beeps_led[beep_code][0][2]));
			//pixels.setPixelColor(i + 1, pixels.Color(beeps_led[beep_code][1][0], beeps_led[beep_code][1][1], beeps_led[beep_code][1][2]));
		}
		//pixels.show();

	}
	if (beep_time == 0) {
		beep_time = millis() + ((beeps_coder[beep_code] & 1) ? LONG_BEEP : SHORT_BEEP);
		beep_bit_n = 0;
		//digitalWrite(BUZZER, HIGH&do_sound);
	}
	else {
		if (beep_bit_n & 1) {
			//pause
		   if(millis() > beep_time) {
				//digitalWrite(BUZZER, HIGH&do_sound);
				beep_bit_n++;
				beep_time = millis() + (((beeps_coder[beep_code] >> (beep_bit_n >> 1)) & 1) ? LONG_BEEP : SHORT_BEEP);
			}
		}
		else {
			//beep
if(millis() > beep_time) {
				//digitalWrite(BUZZER, LOW);
				beep_bit_n++;
				if (beep_bit_n >= 8) {
					beep_code = 0;
					beep_time = 0;
					old_beep_code = 255;
					for (int i = 0; i < 8; i++) ;//pixels.setPixelColor(i, pixels.Color(pi_copter_color[i][0], pi_copter_color[i][1], pi_copter_color[i][2]));
					//pixels.show();
				}
				else
					beep_time = millis() + PAUSE_TIME;
			}
		}
	}

}

void stop_motors() {
	pwmWrite(M0, pwm_OFF_THROTTLE);
	pwmWrite(M1, pwm_OFF_THROTTLE);
	pwmWrite(M2, pwm_OFF_THROTTLE);
	pwmWrite(M3, pwm_OFF_THROTTLE);
}



void throttle_0(const float n) {
	pwmWrite(M0, pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE));
}
void throttle_1(const float n) {
	pwmWrite(M1, pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE));
} 

void throttle_2(const float n) {
	pwmWrite(M2, pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE));
}
void throttle_3(const float n) {
	pwmWrite(M3, pwm_OFF_THROTTLE + (uint16_t)(n*pwm_OFF_THROTTLE));
}


enum COMMANDS_BIT { SOUND_ON = 1, BEEP_CODE = 30 };


volatile uint32_t cnt_reset = 0;
volatile uint8_t cnt_rec = 0;
volatile uint8_t cnt_req = 0;

volatile bool beep_on = false;


uint8_t old_cnt_res = 255, old_cnt_req = 255;
uint16_t err = 0;
int temp;

char inBuf[32];
volatile uint8_t reg = 15;
void receiveEvent(int countToRead) {
	cnt_rec++;
	if (countToRead == 1) {
		reg = Wire.read();
		return;
	}


	//int av = Wire.available();
	Wire.readBytes(inBuf, countToRead);
	
	switch (inBuf[0] & 7)
	{
	case 0:
		{
			uint8_t len = inBuf[0] >> 3;
			uint16_t temp = *((uint16_t*)&inBuf[1]);
			if (len == 0) {
				pwmWrite(M0, constrain(temp, pwm_OFF_THROTTLE, pwm_MAX_THROTTLE));
				bool pow_on = temp > pwm_OFF_THROTTLE;
				temp = *((uint16_t*)&inBuf[3]);
				pow_on |= temp > pwm_OFF_THROTTLE;
				pwmWrite(M1, constrain(temp, pwm_OFF_THROTTLE, pwm_MAX_THROTTLE));
				temp = *((uint16_t*)&inBuf[5]);
				pow_on |= temp > pwm_OFF_THROTTLE;
				pwmWrite(M2, constrain(temp, pwm_OFF_THROTTLE, pwm_MAX_THROTTLE));
				temp = *((uint16_t*)&inBuf[7]);
				pow_on |= temp > pwm_OFF_THROTTLE;
				pwmWrite(M3, constrain(temp, pwm_OFF_THROTTLE, pwm_MAX_THROTTLE));
			}
			else {
				pwmWrite(G_PITCH, constrain(temp, pwm_OFF_THROTTLE, pwm_MAX_THROTTLE));
				temp = *((uint16_t*)&inBuf[3]);
				pwmWrite(G_ROLL, constrain(temp, pwm_OFF_THROTTLE, pwm_MAX_THROTTLE));
				//Serial.print(OCR_GR); Serial.print(" "); Serial.println(OCR_GP);

			}
			break;
		}
	case 1:
		{
			if (inBuf[1] >= 16) {
				if (inBuf[1] == 16) {
					//digitalWrite(SIM800_RESET, LOW);
					cnt_reset = millis() + 10;
				
				}
			}
			else
				beep_code = inBuf[1];
			//Serial.println(beep_code);
			break;
		}
	case 2: 
		{
			overloadVal = *((uint16_t*)&inBuf[1]);
			//printf(overloadVal);
			//printf(inBuf[3]);
			for(int i = 0 ; i < 4 ; i++) {
				printf((int)inBuf[i + 3]);
				if (inBuf[i + 3]) {
					//EEPROM.write(ESC_CALIBR_ADR0 + i, inBuf[i + 3]);
				}
			}
		




			break;
		}
	case 3: 
		{
			new_colors_i = inBuf[0] >> 3;
			pi_copter_color[new_colors_i - 1][0] = *(uint8_t*)&inBuf[1];
			pi_copter_color[new_colors_i - 1][1] = *(uint8_t*)&inBuf[2];
			pi_copter_color[new_colors_i - 1][2] = *(uint8_t*)&inBuf[3];
		}
	default: //setup
		{
			if (countToRead == 7)
			{
				enum { eNO_CON, eRING };
				uint8_t mask = inBuf[0] >> 3;
				do_sound = (mask & 1);
				memcpy((uint8_t*)col, (uint8_t*)&inBuf[1], 6);
			}
		}
	}
	
}

//	A0	-	3v from balance
//	A1	-	6v form balance
//	A2	-	9v form power


volatile float fb[5];

uint16_t readBuffer[5];
char buf[64];

byte gps_buf[12];

volatile uint8_t gps_cnt = 100, old_gps_c = 100;



volatile char ret = 0;
char simbuf[16];
void requestEvent() {



	cnt_req++;


	switch (reg) {
	case 0: 
		{
			readBuffer[0] = fb[0];
			readBuffer[1] = fb[1];
			readBuffer[2] = fb[2];
			readBuffer[3] = fb[3];
			readBuffer[4] = fb[4];
			Wire.write((char*)readBuffer, 10);
			break;
		}
	case 1: 
		{
			//bit 0 - ring; 1 - gps is ready; 2 - m1-overload; 3 -m2-overload ...4,5
			ret = ring_to_send;
			ring_to_send = false;
			if (gps_status == false && gps_cnt != 0 && gps_cnt != old_gps_c) {
				old_gps_c = gps_cnt;
				reg = 2;
				ret |= 2;
			}
			ret |= (overloadF << 2);
			overloadF ^= overloadF;

			Wire.write(ret);
			break;
		}
		//--------------------------------------------------------------
		case 2 : 
			Wire.write((char*)gps_data(), sizeof(SEND_I2C));
		break;
	}
}


//#define ESC_CALIBR
float thr;
float i[5] = { 0, 0, 0, 0, 0 };

int minar = 3000;

bool S4 = true;



void setup()
{

	pinMode(M0, PWM);	
	pinMode(M1, PWM);
	pinMode(M2, PWM);
	pinMode(M3, PWM);	
	pinMode(G_PITCH, PWM);
	pinMode(G_ROLL, PWM);
	
	pinMode(AM0, INPUT_ANALOG);
	pinMode(AM1, INPUT_ANALOG);	
	pinMode(AM2, INPUT_ANALOG);
	pinMode(AM3, INPUT_ANALOG);
	pinMode(BAT, INPUT_ANALOG);
	
	Wire.begin(9);
	Wire.onRequest(requestEvent);    // data request to slave
	Wire.onReceive(receiveEvent);    // data slave received
	
	
	stop_motors();
	
#ifdef PRINT
	Serial.begin(115200);
	//while (!Serial);
	//SIM800.begin(9600);
	printf("HI");

#endif
	//---------------------------------------------------------------
	


	
		overloadVal = 1000;   //shutdown 
		byte esc_calibr[4];

	for (int i = 0; i < 4; i++) 
		if (esc_calibr[i] = EEPROM.read(ESC_CALIBR_ADR0 + i))
			EEPROM.write(ESC_CALIBR_ADR0 + i, 0);
		
	uint16_t MT[4];
	const long t = millis();
	do {
		const long now = millis();
		pwmWrite(M0, MT[0] = (esc_calibr[0] && now < t + 1000 * (long)esc_calibr[0]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		pwmWrite(M1, MT[1] = (esc_calibr[1] && now < t + 1000 * (long)esc_calibr[1]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		pwmWrite(M2, MT[2] = (esc_calibr[2] && now < t + 1000 * (long)esc_calibr[2]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		pwmWrite(M3, MT[3] = (esc_calibr[3] && now < t + 1000 * (long)esc_calibr[3]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		printf(MT[0]);
		printf(MT[1]);
		printf(MT[2]);
		printf(MT[3]);
		delay(1000);
	} while (MT[0] != pwm_OFF_THROTTLE || MT[1] != pwm_OFF_THROTTLE || MT[2] != pwm_OFF_THROTTLE || MT[3] != pwm_OFF_THROTTLE);

	printf("---------");

	gps_setup();
	//pinMode(BUZZER, OUTPUT);
	//pinMode(RING, INPUT);
	//pinMode(SIM800_RESET, OUTPUT);
	//digitalWrite(BUZZER, LOW);
	//digitalWrite(SIM800_RESET, HIGH);
	//analogReference(INTERNAL2V56);

	for(int i = 0 ; i < 100 ; i++)
		fb[4] += ((float)(analogRead(BAT)) - fb[4]) * 0.03;    //volt
	if(fb[4] < (4096.0 / 1.725))
		S4 = false;

	
	//pixels.begin();  // This initializes the NeoPixel library.



	//for(int i = 0 ; i < 8 ; i++)
	//	pixels.setPixelColor(i, pixels.Color(0, 1, 0));  // Moderately bright green pi_copter_color.
	//pixels.show();

	//wdt_enable(WDTO_120MS);
}
///thr0 = m1
//thr1 = m0
//thr2 =
//thr3 =







void loop()
{
	if (cnt_reset > 0 && cnt_reset < millis()) {
		cnt_reset = 0;
		//digitalWrite(SIM800_RESET, HIGH);
	}

	const float CF = 0.01;

	int ar = analogRead(AM0);
	overloadF |= (ar < overloadVal);
	fb[0] += ((float)(ar) - fb[0])*CF;
	ar = analogRead(AM1);
	overloadF |= ((ar < overloadVal) << 1);
	fb[1] += ((float)(ar) - fb[1])*CF;
	ar = analogRead(AM2);
	overloadF |= ((ar < overloadVal) << 2);
	fb[2] += ((float)(ar) - fb[2])*CF;
	ar = analogRead(AM3);
	overloadF |= ((ar < overloadVal) << 3);
	fb[3] += ((float)(ar) - fb[3])*CF;
	
	fb[4] += ((float)(analogRead(BAT)) - fb[4])*CF;    //volt


	if(gps.available()) {
		gps_status = true;
		gps_cnt = processGPS();
		gps_status = false;
	}


	static bool old_pulse = false;
/*
	if (beep_code)
		beep();
	else {

		ring_to_send |= ring = 0;//digitalRead(RING) == LOW;

		if (millis() > 5000 && ring) {
			unsigned long t = millis() & 255;
			//bool puls = t <= 127;
			bool puls = (micros() & (unsigned long)65536) == 0;
			if (puls != old_pulse) {
				old_pulse = puls;
				for (int i = 0; i < 8; i++)
					if (puls)
						pixels.setPixelColor(i, pixels.Color(col[eRING][0], col[eRING][1], col[eRING][2]));   // Moderately bright green pi_copter_color.
				   else
				   	 pixels.setPixelColor(i, pixels.Color(col[eRING][1], col[eRING][0], col[eRING][2]));   // Moderately bright green pi_copter_color.
			//Serial.println(puls);
			pixels.show();
				if (do_sound)
					digitalWrite(BUZZER, puls);
			}
			ring_was = true;
		}
		else 
		{
			static bool old_alarm = false;
			bool alarm = (S4 && fb[4] <= (1200.0 / 1.725));
			if (alarm != old_alarm) {
				digitalWrite(BUZZER, alarm);
				old_alarm = alarm;
			}

		}
	}
	
	//3.3 - alarm ;3.6 red

	//Serial.print(fb[0]); Serial.print(" "); Serial.print(fb[1]); Serial.print(" "); Serial.print(fb[2]); Serial.print(" "); Serial.print(fb[3]); Serial.print(" "); Serial.println(fb[4]);


	if(cnt_req != old_cnt_req && cnt_rec != old_cnt_res) {
		old_cnt_req = cnt_req;
		old_cnt_res = cnt_rec;
		err = 0;

	}
	else {
		err++;
		if (err > 300) {
			stop_motors();
			for (int i = 0; i < 8; i++)
				pixels.setPixelColor(i, pixels.Color(col[eNO_CON][0], col[eNO_CON][1], col[eNO_CON][2]));
			pixels.show();
		}
	}

	if (ring_was && !ring) {
		for (int i = 0; i < 8; i++)
			pixels.setPixelColor(i, pixels.Color(pi_copter_color[i][0], pi_copter_color[i][1], pi_copter_color[i][2]));
		pixels.show();
		ring_was = false;
	}

	if (new_colors_i) {
		new_colors_i--;
		pixels.setPixelColor(new_colors_i, pixels.Color(pi_copter_color[new_colors_i][0], pi_copter_color[new_colors_i][1], pi_copter_color[new_colors_i][2]));
		pixels.show();
		
		new_colors_i = 0;
	}
	wdt_reset();
	*/

	


	//------------------------------------------------------------------------------------------------------------------------------

}
