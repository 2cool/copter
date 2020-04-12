# 1 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
# 1 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"

# 3 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino" 2
# 4 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino" 2
# 5 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino" 2

//#define NUMPIXELS      8

//overflow = 48984






enum
{
 M0 = PA2, M1 = PA6, M2 = PA7, M3 = PA8, G_PITCH = PB0, G_ROLL = PB1, AM0 = PA0, AM1 = PA1, AM2 = PA3, AM3 = PA4, BAT = PA5, BUZZER = PC13, RING = PC14, SIM800_RESET = PC15
}
;

volatile uint8_t beep_code = 0;

uint8_t beeps_coder[] = { 0, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7 }; //4 beeps if 0 short 1 long beep

enum { ESC_CALIBR_ADR0, ESC_CALIBR_ADR1, ESC_CALIBR_ADR2, ESC_CALIBR_ADR3 }
;





uint8_t beeps_led[][2][3]{ {{0,0,0}, {0,0,0}}, { {100,0,0}, {0,0,0}},{{100,0,0}, {0,100,0}},{ {100,0,0}, {0,100,0} },{{100,0,0}, {100,100,0}},{{100,100,0}, {0,0,0}},{{100,100,0}, {0,100,0}},{{100,100,0}, {100,0,0}},{{100,100,0}, {100,100,0}},{{0,0,0}, {0,100,0},},{{0,0,0}, {100,100,0}},{{0,100,0}, {0,0,0}},{{0,100,0}, {0,100,0}},{{0,100,0}, {100,0,0}},{{0,100,0}, {100,100,0}} }
;

volatile bool ring = 0x0, ring_to_send = 0x0;

volatile uint16_t overloadVal = 0;

volatile bool gps_status = 0x0;
volatile uint8_t pi_copter_color[8][3] = { { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 }, { 0, 1, 0 } };

enum { eNO_CON, eRING };
volatile uint8_t col[][3] = { { 1, 0, 0 }, { 255, 0, 0 } };
volatile bool do_sound = 0x1;



volatile uint8_t overloadF = 0;
volatile uint8_t new_colors_i = 0;
bool ring_was = 0x0;





//#define PRINT




# 62 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino" 2

uint8_t beep_bit_n = 0;
uint32 beep_time = 0,buzzer_time=0;
uint8_t old_beep_code = 255;
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
  beep_time = (millis() + ((beeps_coder[beep_code] & 1) ? 600 : 200));
  beep_bit_n = 0;
  digitalWrite(BUZZER, 0x1&do_sound);
 }
 else {
  if (beep_bit_n & 1) {
   //pause
     if(millis() > beep_time) {
    digitalWrite(BUZZER, 0x1&do_sound);
    beep_bit_n++;
    beep_time = millis() + (((beeps_coder[beep_code] >> (beep_bit_n >> 1)) & 1) ? 600 : 200);
   }
  }
  else {
   //beep
   if(millis() > beep_time) {
    digitalWrite(BUZZER, 0x0);
    beep_bit_n++;
    if (beep_bit_n >= 8) {
     beep_code = 0;
     beep_time = 0;
     old_beep_code = 255;
     for (int i = 0; i < 8; i++) ;//pixels.setPixelColor(i, pixels.Color(pi_copter_color[i][0], pi_copter_color[i][1], pi_copter_color[i][2]));
     //pixels.show();
    }
    else
     beep_time = millis() + 100;
   }
  }
 }

}

void stop_motors() {
 pwmWrite(M0, 24002);
 pwmWrite(M1, 24002);
 pwmWrite(M2, 24002);
 pwmWrite(M3, 24002);
}



void throttle_0(const float n) {
 pwmWrite(M0, 24002 + (uint16_t)(n*24002));
}
void throttle_1(const float n) {
 pwmWrite(M1, 24002 + (uint16_t)(n*24002));
}

void throttle_2(const float n) {
 pwmWrite(M2, 24002 + (uint16_t)(n*24002));
}
void throttle_3(const float n) {
 pwmWrite(M3, 24002 + (uint16_t)(n*24002));
}


enum COMMANDS_BIT { SOUND_ON = 1, BEEP_CODE = 30 };


volatile uint32_t cnt_reset = 0;
volatile uint8_t cnt_rec = 0;
volatile uint8_t cnt_req = 0;

volatile bool beep_on = 0x0;


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
    pwmWrite(M0, ((temp)<(24002)?(24002):((temp)>(48004)?(48004):(temp))));
    bool pow_on = temp > 24002;
    temp = *((uint16_t*)&inBuf[3]);
    pow_on |= temp > 24002;
    pwmWrite(M1, ((temp)<(24002)?(24002):((temp)>(48004)?(48004):(temp))));
    temp = *((uint16_t*)&inBuf[5]);
    pow_on |= temp > 24002;
    pwmWrite(M2, ((temp)<(24002)?(24002):((temp)>(48004)?(48004):(temp))));
    temp = *((uint16_t*)&inBuf[7]);
    pow_on |= temp > 24002;
    pwmWrite(M3, ((temp)<(24002)?(24002):((temp)>(48004)?(48004):(temp))));
   }
   else {
    pwmWrite(G_PITCH, ((temp)<(24002)?(24002):((temp)>(48004)?(48004):(temp))));
    temp = *((uint16_t*)&inBuf[3]);
    pwmWrite(G_ROLL, ((temp)<(24002)?(24002):((temp)>(48004)?(48004):(temp))));
   }
   break;
  }
 case 1:
  {
   if (inBuf[1] >= 16) {
    if (inBuf[1] == 16) {
     digitalWrite(SIM800_RESET, 0x0);
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
   ring_to_send = 0x0;
   if (gps_status == 0x0 && gps_cnt != 0 && gps_cnt != old_gps_c) {
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

bool S4 = 0x1;



uint16 setTimer(int n) {
 HardwareTimer timer(n);

 uint32 microseconds = 2041;
 if(!microseconds) {
  timer.setPrescaleFactor(1);
  timer.setOverflow(1);
  return timer.getOverflow();
 }

 uint32 period_cyc = microseconds * (72000000L / 1000000U);
 uint16 prescaler = (uint16)(period_cyc / ((1 << 16) - 1) + 1);
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
 digitalWrite(BUZZER, 0x0);
 digitalWrite(SIM800_RESET, 0x1);
# 366 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
 //---------------------------------------------------------------




 overloadVal = 1000; //shutdown 
 uint16_t esc_calibr[4];

 for (int i = 0; i < 4; i++)
  esc_calibr[i] = 0;

 uint16_t MT[4];
 const long t = millis();
 do {
  const long now = millis();
  pwmWrite(M0, MT[0] = (esc_calibr[0] && now < t + 1000 * (long)esc_calibr[0]) ? 48004 : 24002);
  pwmWrite(M1, MT[1] = (esc_calibr[1] && now < t + 1000 * (long)esc_calibr[1]) ? 48004 : 24002);
  pwmWrite(M2, MT[2] = (esc_calibr[2] && now < t + 1000 * (long)esc_calibr[2]) ? 48004 : 24002);
  pwmWrite(M3, MT[3] = (esc_calibr[3] && now < t + 1000 * (long)esc_calibr[3]) ? 48004 : 24002);

  delay(1000);
 } while (MT[0] != 24002 || MT[1] != 24002 || MT[2] != 24002 || MT[3] != 24002);

 gps_setup();
 for(int i = 0 ; i < 100 ; i++)
  fb[4] += ((float)(analogRead(BAT)) - fb[4]) * 0.03; //volt
 if(fb[4] < (1525))
  S4 = 0x0;

 //pixels.begin();  // This initializes the NeoPixel library.



 //for(int i = 0 ; i < 8 ; i++)
 //	pixels.setPixelColor(i, pixels.Color(0, 1, 0));  // Moderately bright green pi_copter_color.
 //pixels.show();

 //wdt_enable(WDTO_120MS);


 Wire.begin(9);
 Wire.onRequest(requestEvent); // data request to slave
 Wire.onReceive(receiveEvent); // data slave received
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

 fb[4] += ((float)(analogRead(BAT)) - fb[4])*CF; //volt

 unsigned long t = millis();
 if (t - old_t >= 2) {
  old_t = t;
  const int av = Serial3.available();
  if (av) {
   gps_status = 0x1;
   gps_cnt = processGPS();
   gps_status = 0x0;
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
# 472 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
   }
  }







 }

 static bool old_pulse = 0x0;

 if (beep_code)
  beep();
 else {

  ring_to_send |= ring = digitalRead(RING) == 0x0;

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
# 506 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
    if (do_sound)
     digitalWrite(BUZZER, puls);
   }
   ring_was = 0x1;
  }
  else
  {
   static bool old_alarm = 0x0;
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
# 531 "C:\\projects\\copter_stm32\\sketches\\copter_stm32.ino"
  ring_was = 0x0;
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
