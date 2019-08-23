// Settings.h
#include "debug.h"
#include "mpu.h"

#ifndef _SETTINGS_h
#define _SETTINGS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#define HCM_HASH 0
#define HMC_CALIBR 4
#define MOTOR_COMPAS_HASH 16
#define MOTOR_COMPAS 20



//37



#define COMPAS_S 'm'
#define MPU_S 'm'






class SettingsClass
{
 protected:

	 void writeBuf(uint8_t adr, float buf[], uint8_t len);
	 void readBuf(uint8_t adr, float buf[], uint8_t len);
	 void writeBuf(uint8_t adr, int16_t buf[], uint8_t len);
	 void readBuf(uint8_t adr, int16_t buf[], uint8_t len);


 public: 
	 bool load_(string msg, bool any_change);//settings
	 bool readCompasMotorSettings(float base[]);
	 bool saveCompasMotorSettings(float base[]);
	
	 bool saveCompssSettings(int16_t sh[]);


	 bool readCompassSettings(int16_t sh[]);

	 bool any_change = false;
	 void set(const float  val, float &set);
public:
	void write(int i, char c);
	char read(int i);
	int write();
	int read();

	int write_all();
	int read_all();

	void init();









};


	
extern SettingsClass Settings;

#endif

