#include "Log.h"
#include "debug.h"
#include "Autopilot.h"


#define LOG_VER "log001"

volatile  bool	run_loging = true;
volatile int log_index, log_bank_, log_bank, old_bank,net_bank, error_bansk = 0;
volatile bool log_file_closed=true;

volatile int counter = 0;

#define max_blocks_2_send 4
#define mask 127
uint8_t log_buffer[mask+1][1024];


ofstream logfile;
string this_log_fname;

void LogClass::write_bank_cnt() {
	uint8_t *fp = (uint8_t*)&log_bank_;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];

}
uint8_t * LogClass::getNext(int &len) {

	if (true){//Mpu.timed - Autopilot.last_time_data_recived > 0.5 || run_loging == false || log_bank_ <= net_bank) {
		len = 0;
		return 0;
	}

	if (log_bank_ - net_bank > max_blocks_2_send) {
		net_bank = log_bank_ - max_blocks_2_send;
		//printf("net_err\n");
	}


	len = *((uint16_t*)log_buffer[net_bank & mask]);
	uint8_t *ret=log_buffer[net_bank & mask];
	net_bank++;
	return ret;
}
void loger() {
	while (run_loging) {
		while (run_loging && log_bank_ <= old_bank) {
			usleep(1000);
		}
		
		//printf("%i\n",log_bank_ - old_bank);
		
		if (log_bank_ - old_bank > mask) {
			cout << "log sync error! %i\n", log_bank_;
			error_bansk += log_bank_ - old_bank - mask;
			old_bank = log_bank_ - mask;
			
		}
		
		if (logfile.is_open()) {
			int len = *((uint16_t*)log_buffer[old_bank & mask]);
			logfile.write((char*)log_buffer[old_bank & mask], len);
			logfile.flush();
			old_bank++;
		}else{
			cout << "LOG ERROR\n";
		}
			
		
	}
	logfile.close();



	//remove old logs

	int errors = 0;

	for (int cnt = counter - 20;  cnt>10000; cnt--) {
		ostringstream c_log, c_txt;
		c_log << "/home/igor/logs/tel_log" << cnt<< ".log";
		c_txt << "/home/igor/logs/log_out" << cnt << ".txt";
		string t_log = c_log.str();
		string t_txt = c_txt.str();

		errors+=(remove(t_log.c_str())!=0);
		(remove(t_txt.c_str())!=0);
		if (errors > 50)
			break;
	}


	
	log_file_closed = true;


}

int LogClass::counter_() { return counter; }

bool LogClass::close() {
	if (writeTelemetry) {
		cout << "close tel log\n";
		cout << "banks: "<< log_bank_ <<"\twrited banks: "<< old_bank <<"\terrors:"<< error_bansk <<endl;
		run_loging = false;
		while (log_file_closed == false)
			usleep(100000);
		return true;
	}
	return true;



}

bool LogClass::init(int counter_) {
	counter = counter_;
	run_counter = 0;
	if (writeTelemetry) {
		run_loging = true;
		ostringstream convert;
		convert << "/home/igor/logs/tel_log" << counter << ".log";
		this_log_fname = convert.str();
		cout << "log 2 " << this_log_fname << endl;
		logfile.open(this_log_fname.c_str(), fstream::in | fstream::out | fstream::trunc);

		//char ver[] = LOG_VER;
		//logfile.write(ver, 6);

		log_file_closed = false;
		log_bank_ = old_bank = log_bank = net_bank=0;
		log_index = 2;
		thread t(loger);
		t.detach();
	}
	else
		log_file_closed = true;
	return logfile.is_open();
}
void LogClass::loadFloat(float f) {
	uint8_t *fp = (uint8_t*)&f;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
}

void LogClass::loaduint64t(uint64_t ui) {
	uint8_t *fp = (uint8_t*)&ui;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
	log_buffer[log_bank][log_index++] = fp[4];
	log_buffer[log_bank][log_index++] = fp[5];
	log_buffer[log_bank][log_index++] = fp[6];
	log_buffer[log_bank][log_index++] = fp[7];
}

void LogClass::loaduint32t(uint32_t ui) {
	uint8_t *fp = (uint8_t*)&ui;
	log_buffer[log_bank][log_index++] = fp[0];
	log_buffer[log_bank][log_index++] = fp[1];
	log_buffer[log_bank][log_index++] = fp[2];
	log_buffer[log_bank][log_index++] = fp[3];
}
void LogClass::loadInt16t(int16_t i) {
	uint8_t *ip = (uint8_t*)&i;
	log_buffer[log_bank][log_index++] = ip[0];
	log_buffer[log_bank][log_index++] = ip[1];
}


void LogClass::loadSEND_I2C(SEND_I2C *p) {
	memcpy((uint8_t*)&log_buffer[log_bank][log_index], p, sizeof(SEND_I2C));
	log_index += sizeof(SEND_I2C);
}

void LogClass::loadMem(uint8_t*src, int len,bool write_mem_size) {
	if (write_mem_size) {
		uint8_t *fp = (uint8_t*)&len;
		log_buffer[log_bank][log_index++] = fp[0];
		log_buffer[log_bank][log_index++] = fp[1];
	}
	memcpy(&(log_buffer[log_bank][log_index]), src, len);
	log_index += len;
}

static int start_block;
void LogClass::block_start(int type, bool two_byte_size_block) {
	loadByte(type);
	start_block = log_index;
	if (two_byte_size_block) {
		log_buffer[log_bank][log_index] = 0;
		log_index += 3;
	}
	else {
		log_index++;
	}
}
void LogClass::block_end(bool two_byte_size_block) {
	if (two_byte_size_block) {
		int i = (log_index - start_block-1);
		uint8_t *ip = (uint8_t*)&i;
		log_buffer[log_bank][start_block] = ip[0];
		log_buffer[log_bank][start_block+1] = ip[1];
	}
	else {
		log_buffer[log_bank][start_block] = log_index - start_block-1;
	}
}

void LogClass::loadByte(uint8_t b)
{
	log_buffer[log_bank][log_index++] = b;
}

int max_log_index = 0;
void LogClass::end() {
	//if (log_index > max_log_index)
	//{
	//	max_log_index = log_index;
	//	printf("max log index=%i\n", log_index);
	//}
	if (log_index > 2) {
		uint8_t *fp = (uint8_t*)&log_index;
		log_buffer[log_bank][0] = fp[0];
		log_buffer[log_bank][1] = fp[1];
		log_bank_++;
		log_bank = log_bank_ & mask;

		log_index = 2;
	}
}

LogClass Log;