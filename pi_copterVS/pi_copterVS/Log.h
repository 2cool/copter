

#ifndef _LOG_h
#define _LOG_h

#include "define.h"
#include "Location.h"


/*
mpu=34b
gps=11b
hmc=17b
MS5=6b
comm=1024

*/
enum LOG { MPU_EMU,MPU_SENS,HMC_BASE,HMC_SENS, HMC,GPS_SENS, TELE,COMM,    EMU,AUTO,BAL,MS5611_SENS,XYSTAB,ZSTAB};
class LogClass
{
private:
	
	
public:
	void write_bank_cnt();
	uint8_t * getNext(int &len);
	bool writeTelemetry;
	int counter_();
	int run_counter;
	void end();
	void loadSEND_I2C(SEND_I2C *p);


	void loaduint64t(uint64_t ui);
	void loaduint32t(uint32_t ui);
	void loadFloat(float f);
	void loadInt16t(int16_t i);
	void loadMem(uint8_t*buf, int len,bool write_mem_size=true);
	void loadByte(uint8_t b);
	void block_start(int type, bool two_byte_size_block=false);
	void block_end(bool two_byte_size_block = false);
	bool init(int counter);
	bool close();
};

extern LogClass Log;

#endif