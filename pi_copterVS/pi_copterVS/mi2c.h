// mega_i2c.h


/*

timer 	bits 	channel 	Arduino pin 	Mega pin
timer0 	8 		A 			6				13
timer0 	8 		B 			5				4
timer1 	16 		A 			9 				11
timer1 	16 		B 			10				12
timer2 	8 		A 			11			 	10
timer2 	8 		B 			3				9
timer3 	16 		A 			-				5		x
timer3 	16 		B 			-				2
timer3 	16 		C 			-				3		x
timer4 	16 		A 			-				6		x
timer4 	16 		B			- 				7		buzzer
timer4 	16		C			- 				8		x
timer5 	16 		A 			- 				44		ginbal
timer5 	16 		B 			- 				45		ginbal
timer5 	16 		C 			- 				46












^
0   |    1
\  |  /
\   /
000
000
/   \
/     \
2       3

*/

#ifndef _PWM_h
#define _PWM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*
������� - ���� � �������
1600 33
1940 97
2280 170
2620 249
2960 341
3300 450
3640 520
3980 620
4320 700
4660 750
*/



#include "define.h"


class Megai2c
{
protected:
	int current_led_mode;
public:
	uint8_t  DO_SOUND = 0;
	void set_led_mode(uint8_t n,uint8_t briht, bool pulse=false);
	uint8_t motors_overload = 0;
	 bool ring_received;
	
	 int init();


	 bool gimagl(float pitch, float roll);
	 void set_led_color(uint8_t n, uint8_t r, uint8_t g, uint8_t b);  // n form 1 to 8
	 void sound(const float);
	 void beep_code(uint8_t);
	 void sim800_reset();
	 
	 //esc_calibr = time betwen max_throtthle and zero_throtthle is seconds If zero no calibration it work in next power on;
	 void settings(float overCurrentVal, uint8_t esc_calibr0, uint8_t esc_calibr1, uint8_t esc_calibr2, uint8_t esc_calibr3);
	 void throttle(const float n[]);

	 int getiiiiv(char *iiiiv);

	 int get_gps(SEND_I2C *gps_d);

	 void Buzzer(const bool on);

	
	
	// int send2sim(const char *str, int len);
	// int getsim(char * str);

private:
	
	 void m_parser(char *str, int len);
	 uint16_t correct(const float n);

};

extern Megai2c mega_i2c;

#endif

