#include <sys/types.h>
#include "WProgram.h"
#include "define.h"

#include <sys/shm.h>
#include <sys/ipc.h>
#include "mpu.h"
#include <chrono>



/*
 static __time_t start_seconds;
long sec;
long usec;
 void  init_millis_micros() {

	 auto start = chrono::steady_clock::now();


	 timespec t;
	 clock_gettime(CLOCK_REALTIME, &t);
	 sec = t.tv_sec;
	 usec = t.tv_nsec;
#ifdef FLY_EMULATOR
	 start_seconds = t.tv_sec - 30L;
#else
	 start_seconds = t.tv_sec-1;
#endif
	
 }


 void  correct_start_time(const __time_t sec) {
	 start_seconds += sec;

 }



 __time_t get_start_sec() { return start_seconds; }
uint32_t millis_g_() {
	timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	uint32_t ret;
	
	ret = (uint32_t)(((t.tv_sec ) * 1000L) + (t.tv_nsec / 1000000L));
	return ret;
}
uint32_t millis_(){
	//return chrono::duration_cast<chrono::milliseconds>(end - start).count();
	
	timespec t;
	clock_gettime(CLOCK_REALTIME,&t);
	uint32_t ret;
	ret=(uint32_t)(((t.tv_sec-start_seconds)*1000L)+(t.tv_nsec/1000000L));
	return ret;
	
}



int64_t micros_(void){
	timespec t;
	clock_gettime(CLOCK_REALTIME,&t);
	int64_t ret;
	ret=((int64_t)(t.tv_sec-start_seconds)*1000000L)+(t.tv_nsec/1000L);
	sec = t.tv_sec;
	usec = t.tv_nsec;
	return ret;
}

*/

static auto start = chrono::steady_clock::now();

double uptime(bool init) {
	if (init)
		;// start = chrono::high_resolution_clock::now();
	auto end = chrono::steady_clock::now();
	uint64_t s = (double)start.time_since_epoch().count();
	uint64_t e = (double)end.time_since_epoch().count();
	return double(e-s)* 0.000000001;
}
uint64_t micros_() {

	auto end = chrono::steady_clock::now();
	return chrono::duration_cast<chrono::milliseconds>(end - start).count();
}
uint32_t millis_() {
	auto end = chrono::steady_clock::now();
	return (uint32_t)chrono::duration_cast<chrono::milliseconds>(end - start).count();
}

void delay(unsigned long t){
	usleep(t*1000);
}

std::string exec(const std::string cmd) {
	//printf(cmd.c_str());
	//printf("\n");
	char buffer[128];
	std::string result = "";
	//cout << exec << endl;
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) {
		//throw std::runtime_error("popen() failed!");
		cout << "pipe brock" << "\t"<<Mpu.timed << endl;
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
			cout << "*** shmget error (server) ***" << "\t"<<Mpu.timed << endl;
			return 1;
		}
		shmPTR = (struct Memory *) shmat(ShmID, NULL, 0);
	}
	return 0;
}

void close_shmPTR() {
	cout << "Server has detected the completion of its child..." << "\t"<<Mpu.timed << endl;
	shmdt((void *)shmPTR);
	cout << "Server has detached its shared memory..." << "\t"<<Mpu.timed << endl;
	shmctl(ShmID, IPC_RMID, NULL);
	cout << "Server has removed its shared memory..." << "\t"<<Mpu.timed << endl;
	cout << "Server exits..." << "\t"<<Mpu.timed << endl;
}
