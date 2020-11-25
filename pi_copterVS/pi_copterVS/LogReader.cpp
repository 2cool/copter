#include "LogReader.h"

#include <sys/sem.h>
#include  <sys/types.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>

#include <unistd.h>



#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>


using namespace std;


typedef unsigned char byte;

LogReader::LogReader() {
	sd.done = 0;
	
	
}

int load_uint8_(byte buf[], int i) {
	int vall = buf[i];
	vall &= 255;
	return vall;
}
int16_t load_int16_(byte buf[], int i) {
	int16_t* ip = (int16_t*)&buf[i];
	return *ip;
}
int32_t load_int32(byte buf[], int i) {
	int32_t* ip = (int32_t*)&buf[i];
	return *ip;
}
uint64_t loaduint64t(byte buf[], int i) {
	uint64_t* ip = (uint64_t*)&buf[i];
	return *ip;

}

//string fname = "ddd";
char* fname="/home/igor/logs/tel_log_real13098.log";

int i = 0;
byte* buffer=NULL;
long lSize = 0;
//----------------------------------------------------
int LogReader::readLog() {

	FILE* pFile;
	size_t result;


	pFile = fopen(fname, "rb");
	if (pFile == NULL) { fputs("File error", stderr); exit(1); }

	// obtain file size:
	fseek(pFile, 0, SEEK_END);
	lSize = ftell(pFile);
	rewind(pFile);

	// allocate memory to contain the whole file:
	buffer = (byte*)malloc(sizeof(char) * lSize);
	if (buffer == NULL) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	result = fread(buffer, 1, lSize, pFile);
	if (result != lSize) { fputs("Reading error", stderr); exit(3); }

	fclose(pFile);

	return lSize;// decode_Log();
}

void LogReader::mpu_parser() {
	static uint64_t old_itime = 0;
	uint64_t itime = loaduint64t(buffer, i);
	sd.time = itime * 1000;	
	set_current_time(itime * 1000);
	memcpy((uint8_t*)sd.g, buffer + i+8, 6);
	memcpy((uint8_t*)sd.a, buffer + i+8+6, 6);
}

void  LogReader::ms5611_parser() {
	sd.i_readTemperature = buffer[i];
	sd.pressure = *(float*)&buffer[i+1];
}

void LogReader::gps_parser() {
	sd.gps = *(SEND_I2C*)&buffer[i];
}

void LogReader::hmc_parser() {
	//Log.loadMem(buffer, 6, false);
	memcmp((uint8_t*)sd.buffer, buffer+i, 6);
}

void LogReader::hmc_parser_base() {
	memcpy((uint8_t*)sd.sh, buffer + i, 12);
	memcpy((uint8_t*)sd.base, buffer + i+12, 12*4);
	sd.yaw_correction_angle = RAD2GRAD * *(float*)&buffer[i + 12 + 12 * 4];
}

void LogReader::telemetry_parser() {
	memcpy((byte*)sd.data, &buffer[i], 10);
}

void LogReader::commander_parser(int len) {

	memcpy((uint8_t*)shmPTR->commander_buf, buffer + i+2, len);
	shmPTR->connected = 1;
	shmPTR->commander_buf_len = len;


}

enum LOG { MPU_EMU, MPU_SENS, HMC_BASE, HMC_SENS, HMC, GPS_SENS, TELE, COMM, EMU, AUTO, BAL, MS5611_SENS, XYSTAB, ZSTAB };

void LogReader::set(const uint8_t bit) {
	sd.done |= (1 << bit);
}
void LogReader::unset(const uint8_t b) {
	sd.done &= ~(1 << b);
}
bool LogReader::test(const uint8_t b) {
	return (sd.done & (1 << b));
}


int j = 0;
int LogReader::parser(const uint8_t need) {
	bool ok = false;

	if (test(need)) {
		unset(need);
		return 1;
	}

	if (buffer == NULL)
		lSize=readLog();
	if (i >= 508000-100)
		i = i;

	if (lSize <= i)
		return 0;
	const int f_len = load_int16_(buffer, i);
	i += 2;
	
	while (i < f_len+j) {
		const int b = buffer[i++];
		int len = load_uint8_(buffer, i);
		if (len == 0 && b == 9)
			len = 4;
		i++;
		if (len == 0) {
			len = load_int16_(buffer, i);
			if (len < 0)
				len = 0;
			i += 2;
		}
		
		switch (b) {

		case MPU_SENS: {
			if (len == 5)
				ms5611_parser();
			else {
				mpu_parser();
			}
			break;
		}
		case MS5611_SENS: {
			ms5611_parser();
			break;
		}
		case GPS_SENS: {
			gps_parser();
			break;
		}

		case AUTO: {
			sd.control_bits = *(uint32_t*)&buffer[i];
			break;
		}
		case HMC_BASE: {
			hmc_parser_base();
			break;

		}
		case HMC_SENS: {
			hmc_parser();
			break;
		}
		case HMC: {
			
			break;
		}
		case TELE: {
			telemetry_parser();
			break;
		}
		case ZSTAB: {
			
			break;
		}
		case BAL: {
			break;
			break;
		}
		case COMM: {
			commander_parser(len);
			break;
		}
		default:

			break;
		}
		if (need == b) {
			ok = true;
			unset(b);
		}
		else
			set(b);
		i += len;

	}
	j += f_len;
	
	
	return ok;
}









LogReader logR;

/*
chitat' vse

*/