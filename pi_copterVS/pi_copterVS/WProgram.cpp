#include <sys/types.h>
#include "WProgram.h"
#include "define.h"
#include "ssd1306.h"
#include <sys/shm.h>
#include <sys/ipc.h>
#include "mpu.h"
#include <chrono>


#ifdef LOG_READER
long log_time = 0;

void set_current_time(long t) {
	long log_time = t;
}
int64_t micros_() {
	return log_time * 1000;
}
int32_t millis_() {
	return log_time;
}


#else



std::chrono::duration<long, std::milli> error_tvv{ 3070000 };
static auto start = chrono::steady_clock::now();// -error_tvv;

int64_t micros_() {

	auto end = chrono::steady_clock::now();
	return chrono::duration_cast<chrono::microseconds>(end - start).count();
}
int32_t millis_() {
	auto end = chrono::steady_clock::now();
	return (int32_t)chrono::duration_cast<chrono::milliseconds>(end - start).count();
}

void delay(unsigned long t){
	usleep(t*1000);
}
#endif



std::string exec(const std::string cmd) {
	//printf(cmd.c_str());
	//printf("\n");
	char buffer[128];
	std::string result = "";
	//cout << exec << endl;
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) {
		//throw std::runtime_error("popen() failed!");
		cout << "pipe brock" << "\t"<<millis_() << endl;
		return "";
	}
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}
	pclose(pipe);
	return result;
}


key_t          ShmKEY;
int            ShmID;
struct Memory *shmPTR;

int init_shmPTR() {
	if (shmPTR == 0) {


		ShmKEY = ftok(SHMKEY, 'x');
	ShmID = shmget(ShmKEY, sizeof(struct Memory), IPC_CREAT | 0666);
		if (ShmID < 0) {
			cout << "***main shmget error (server) ***" << "\t"<<millis_() << endl;
			myDisplay.textDisplay("main shmget error\n");
			return 1;
		}
		shmPTR = (struct Memory *) shmat(ShmID, NULL, 0);
	}
	return 0;
}

void close_shmPTR() {
	cout << "Server has detected the completion of its child..." << "\t"<<millis_() << endl;
	shmdt((void *)shmPTR);
	cout << "Server has detached its shared memory..." << "\t"<<millis_() << endl;
	shmctl(ShmID, IPC_RMID, NULL);
	cout << "Server has removed its shared memory..." << "\t"<<millis_() << endl;
	cout << "Server exits..." << "\t"<<millis_() << endl;
}
