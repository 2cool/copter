#line 1 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
#line 1 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"

#include <stdint.h>
#include <Arduino.h>
#include <Wire_slave.h>

//#define NUMPIXELS      8

//overflow = 48984
#define MAX90_THROTTLE   45604
#define pwm_MIN_THROTTLE 26402
#define pwm_MAX_THROTTLE 48004
#define pwm_OFF_THROTTLE 24002


enum
{
	M0 = PA2, M1 = PA6, M2 = PA7, M3 = PA8, G_PITCH = PB0, G_ROLL = PB1, AM0 = PA0, AM1 = PA1, AM2 = PA3, AM3 = PA4, BAT = PA5, BUZZER = PC13, RING = PC14, SIM800_RESET = PC15
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

//#define PRINT




#include "gps.h"

uint8_t beep_bit_n = 0;
uint32 beep_time = 0,buzzer_time=0;
uint8_t old_beep_code = 255;
#line 66 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void beep();
#line 113 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void stop_motors();
#line 122 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_0(const float n);
#line 125 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_1(const float n);
#line 129 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_2(const float n);
#line 132 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void throttle_3(const float n);
#line 153 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void receiveEvent(int countToRead);
#line 260 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void requestEvent();
#line 312 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
uint16 setTimer(int n);
#line 330 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void setup();
#line 421 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void loop();
#line 66 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
void beep() {

	if (old_beep_code != beep_code)
	{
		old_beep_code = beep_code;
		//for (int i = 0; i < 8; i += 2)
		//{
			//pixels.setPixelColor(i, pixels.Color(beeps_led[beep_code][0][0], beeps_led[beep_code][0][1], beeps_led[beep_code][0][2]));
			//pixels.setPixelColor(i + 1, pixels.Color(beeps_led[beep_code][1][0], beeps_led[beep_code][1][1], beeps_led[beep_code][1][2]));
		//}
		//pixels.show();

	}
	if (beep_time == 0) {
		beep_time = (millis() + ((beeps_coder[beep_code] & 1) ? LONG_BEEP : SHORT_BEEP));
		beep_bit_n = 0;
		digitalWrite(BUZZER, HIGH&do_sound);
	}
	else {
		if (beep_bit_n & 1) {
			//pause
		   if(millis() > beep_time) {
				digitalWrite(BUZZER, HIGH&do_sound);
				beep_bit_n++;
				beep_time = millis() + (((beeps_coder[beep_code] >> (beep_bit_n >> 1)) & 1) ? LONG_BEEP : SHORT_BEEP);
			}
		}
		else {
			//beep
			if(millis() > beep_time) {
				digitalWrite(BUZZER, LOW);
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
			}
			break;
		}
	case 1:
		{
			if (inBuf[1] >= 16) {
				if (inBuf[1] == 16) {
					digitalWrite(SIM800_RESET, LOW);
					cnt_reset = millis() + 10;
				
				}
			}
			else
				beep_code = inBuf[1];
			break;
		}
	case 2: 
		{
			overloadVal = *((uint16_t*)&inBuf[1]);
			//printf(overloadVal);
			//printf(inBuf[3]);
			for(int i = 0 ; i < 4 ; i++) {
				//printf((int)inBuf[i + 3]);
				//if (inBuf[i + 3]) {
				//	EEPROM.write(ESC_CALIBR_ADR0 + i, inBuf[i + 3]);
				//}
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

#define MAX_RELOAD ((1 << 16) - 1)

uint16 setTimer(int n) {
	HardwareTimer timer(n);
	
	uint32 microseconds = 2041;
	if(!microseconds) {
		timer.setPrescaleFactor(1);
		timer.setOverflow(1);
		return timer.getOverflow();
	}

	uint32 period_cyc = microseconds * CYCLES_PER_MICROSECOND;
	uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
	uint16 overflow = (uint16)((period_cyc + (prescaler / 2)) / prescaler);
	timer.setPrescaleFactor(prescaler);
	timer.setOverflow(overflow);
	return overflow;

}
void setup()
{

	setTimer(1);
	setTimer(2);
	setTimer(3);
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
	

	
	stop_motors();
	pinMode(BUZZER, OUTPUT);	
	pinMode(RING, INPUT);
	pinMode(SIM800_RESET, OUTPUT);
	digitalWrite(BUZZER, LOW);
	digitalWrite(SIM800_RESET, HIGH);
	
	
#ifdef PRINT
	Serial.begin(115200);
	//while (!Serial);
	//SIM800.begin(9600);
	Serial.println("HI");

#endif
	//---------------------------------------------------------------
	


	
	overloadVal = 1000;   //shutdown 
	uint16_t esc_calibr[4];

	for (int i = 0; i < 4; i++) 
		esc_calibr[i] = 0;
		
	uint16_t MT[4];
	const long t = millis();
	do {
		const long now = millis();
		pwmWrite(M0, MT[0] = (esc_calibr[0] && now < t + 1000 * (long)esc_calibr[0]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		pwmWrite(M1, MT[1] = (esc_calibr[1] && now < t + 1000 * (long)esc_calibr[1]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		pwmWrite(M2, MT[2] = (esc_calibr[2] && now < t + 1000 * (long)esc_calibr[2]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);
		pwmWrite(M3, MT[3] = (esc_calibr[3] && now < t + 1000 * (long)esc_calibr[3]) ? pwm_MAX_THROTTLE : pwm_OFF_THROTTLE);

		delay(1000);
	} while (MT[0] != pwm_OFF_THROTTLE || MT[1] != pwm_OFF_THROTTLE || MT[2] != pwm_OFF_THROTTLE || MT[3] != pwm_OFF_THROTTLE);

	gps_setup();
	for(int i = 0 ; i < 100 ; i++)
		fb[4] += ((float)(analogRead(BAT)) - fb[4]) * 0.03;    //volt
	if(fb[4] < (1525))
		S4 = false;

	//pixels.begin();  // This initializes the NeoPixel library.



	//for(int i = 0 ; i < 8 ; i++)
	//	pixels.setPixelColor(i, pixels.Color(0, 1, 0));  // Moderately bright green pi_copter_color.
	//pixels.show();

	//wdt_enable(WDTO_120MS);
	
	
	Wire.begin(9);
	Wire.onRequest(requestEvent);     // data request to slave
	Wire.onReceive(receiveEvent);     // data slave received
}
///thr0 = m1
//thr1 = m0
//thr2 =
//thr3 =







void loop()
{
	static unsigned long old_t = 0;
	if (cnt_reset > 0 && cnt_reset < millis()) {
		cnt_reset = 0;
		//digitalWrite(SIM800_RESET, HIGH);
	}

	const float CF = 0.001;

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

	unsigned long t = millis();
	if (t - old_t >= 2)	{
		old_t = t;
		const int av = gps.available();
		if (av) {
			gps_status = true;
			gps_cnt = processGPS();
			gps_status = false;
		}
		
		
		
		if (cnt_req != old_cnt_req && cnt_rec != old_cnt_res) {
			old_cnt_req = cnt_req;
			old_cnt_res = cnt_rec;
			err = 0;
		}
		else {
			err++;
			if (err > 200) {
				stop_motors();
				/*
				for (int i = 0; i < 8; i++)
					pixels.setPixelColor(i, pixels.Color(col[eNO_CON][0], col[eNO_CON][1], col[eNO_CON][2]));
		pixels.show();
		*/
			}
		}
		
		
		
		
		
		
		
	}

	static bool old_pulse = false;

	if (beep_code)
		beep();
	else {

		ring_to_send |= ring = digitalRead(RING) == LOW;

		if (millis() > 5000 && ring) {
			buzzer_time = millis();
			unsigned long t = buzzer_time & 255;
			//bool puls = t <= 127;
			bool puls = (micros() & (unsigned long)65536) == 0;
			if (puls != old_pulse) {
				old_pulse = puls;
				/*
				for (int i = 0; i < 8; i++)
					if (puls)
						pixels.setPixelColor(i, pixels.Color(col[eRING][0], col[eRING][1], col[eRING][2]));   // Moderately bright green pi_copter_color.
				   else
				   	 pixels.setPixelColor(i, pixels.Color(col[eRING][1], col[eRING][0], col[eRING][2]));   // Moderately bright green pi_copter_color.
			pixels.show();
				*/
				if (do_sound)
					digitalWrite(BUZZER, puls);
			}
			ring_was = true;
		}
		else 
		{
			static bool old_alarm = false;
			bool alarm = (S4 && fb[4] <= (2127));
			if (alarm != old_alarm) {
				digitalWrite(BUZZER, alarm);
				old_alarm = alarm;
			}

		}
	}
	
	//3.3 - alarm ;3.6 red

	if (ring_was && !ring) {
		/*
		for (int i = 0; i < 8; i++)
			pixels.setPixelColor(i, pixels.Color(pi_copter_color[i][0], pi_copter_color[i][1], pi_copter_color[i][2]));
		pixels.show();
		*/
		ring_was = false;
	}

	if (new_colors_i) {
		new_colors_i--;
		//pixels.setPixelColor(new_colors_i, pixels.Color(pi_copter_color[new_colors_i][0], pi_copter_color[new_colors_i][1], pi_copter_color[new_colors_i][2]));
		//pixels.show();
		new_colors_i = 0;
	}
	if (buzzer_time && millis() - buzzer_time > 1000) {
		digitalWrite(BUZZER, 0);
		buzzer_time = 0;
	}

	
	//wdt_reset();
	

	


	//------------------------------------------------------------------------------------------------------------------------------

}
