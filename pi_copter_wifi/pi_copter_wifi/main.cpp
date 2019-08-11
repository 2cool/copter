#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <sys/types.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
//#define FORTEST
#include <thread>
#ifdef FORTEST
#include"C:/Users/2coolz/pi_copter_mega/pi_copterVS/pi_copterVS/glob_header.h"
//#include "../../../../repos/pi_copter_mega/pi_copterVS/pi_copterVS/glob_header.h"

#else
#include "../pi_copterVS/glob_header.h"
#endif

key_t          ShmKEY;
int            ShmID;
struct Memory *shmPTR;



void delay(int t) {
	usleep(t * 1000);
}

int init_shmPTR() {
	if (shmPTR == 0) {


		ShmKEY = ftok(SHMKEY, 'x');
		ShmID = shmget(ShmKEY, sizeof(struct Memory), 0666);
		if (ShmID < 0) {
			cout << "*** shmget error (wifi) ***\n";
			return 1;
		}
		shmPTR = (struct Memory *) shmat(ShmID, NULL, 0);
	}
	return 0;
}

string get_client_addres();




//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}
void pipe_handler(int sig) {
	cout << "wifi pipe error\n";
}

int sockfd;
struct sockaddr_in serv_addr, cli_addr;

int server() {


	


	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {

		return -1;
	}
	memset(&serv_addr, 0, sizeof(serv_addr));
	memset(&cli_addr, 0, sizeof(cli_addr));
	// Filling server information 
	serv_addr.sin_family = AF_INET; // IPv4 
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(9876);
	//inet_aton("192.168.100.101", (in_addr*)& servaddr.sin_addr.s_addr);
	// Bind the socket with the server address 
	if (bind(sockfd, (const struct sockaddr*) & serv_addr, sizeof(serv_addr)) < 0)
		return -1;
	//char* buffer = (char*)shmPTR->wifiRbuffer;


	while (flag == 0 && shmPTR->run_main) {
		while(shmPTR->commander_buf_len != 0)
				usleep(10000);
		int n, len;
		n = recvfrom(sockfd, (char*)shmPTR->commander_buf, TELEMETRY_BUF_SIZE, MSG_WAITALL, (struct sockaddr*) & cli_addr, (socklen_t*)& len);


		if (n > 0) {
			//cout << "<<" << n << endl;
			shmPTR->commander_buf_len = n;
			if (shmPTR->connected == 0) {
				shmPTR->client_addr = cli_addr.sin_addr.s_addr;
				shmPTR->connected++;
				cout << "ONLINE\n";
			}

			while (flag == 0 && shmPTR->telemetry_buf_len == 0)
				usleep(10000);
			len = shmPTR->telemetry_buf_len;
			n = sendto(sockfd, (char*)shmPTR->telemetry_buf, len, MSG_CONFIRM, (const struct sockaddr*) & cli_addr, sizeof(cli_addr));

			if (len == n) {
				shmPTR->telemetry_buf_len = 0;
				//cout << ">>" << n << endl;
			}
			else {
				fprintf(stderr, "socket() failed: %s\n", strerror(errno));
				cout << ">>error\n";
			}
		}
		else {
			fprintf(stderr, "socket() failed: %s\n", strerror(errno));
			cout << "<<error\n";
		}
		static int cnt = 0;
		//cout << "--------------------- " << cnt << endl;
		cnt++;

	}
	close(sockfd);

		//////////////////////////

	
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
std::string exec(const std::string cmd) {
	//printf(cmd.c_str());
	//printf("\n");
	char buffer[128];
	std::string result = "";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) {
		//throw std::runtime_error("popen() failed!");
		cout << "wifi: pipe brock\n";
		return "";
	}
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}
	//cout << "close pipe\n";
	pclose(pipe);
	return result;
}


//"*  2coolzNET      Infra  11    54 Mbit/s  93      ▂▄▆█  WPA1 WPA2"
//          Link Quality=5/5  Signal level=-39 dBm  Noise level=-91 dBm

void get_signal_strong() {
	string ret = exec("nice -n -20 iwconfig wlan0 | grep Signal");
	if (ret.length() > 20) {
		int beg = ret.find("l=-");
		int len = ret.substr(beg+3).find("dBm");
		int signal = atoi(ret.substr(beg+3, len).c_str());
		shmPTR->status = signal;
	}
	//string ret = exec("nmcli dev wifi | grep 2coolzNET");
	
}



//----------------------------------------------------------
void test_wifi() {
	int ln = 2;
	cout << "test wifi work\n";
	string ret = exec("nice -n -20 ifconfig wlan0");
	int ip;
	string myIP = "";
	ip = ret.find("192.168.");
	if (ip >= 0)
		myIP = ret.substr(ip, 8 + ret.substr(ip + 8).find(".")) + ".1";
	if (myIP.length() > 0)
		ret = exec("nice -n -20 ping -w 1 -c 1 " + myIP);
	if (ret.find("1 received") == string::npos) {
		if (--ln <= 0) {
			//printf( "%s %i\n", ret.c_str(),n);
			ret = exec("nice -n -20 nmcli dev wifi | grep 2coolzNET");
			if (ret.find(" 2coolzNET ") != string::npos) {
				exec("nice -n -20 ifconfig wlan0 down");
				delay(1000);
				exec("nice -n -20 ifconfig wlan0 up");
				delay(5000);
				ln = 2;
			}
		}
	}
	else {
		ln = 2;
		cout << "test is OK\n";
	}
}
void watch_d() {
	static int cnt_wifi_strong = 0;
	static int errors = 0;
	shmPTR->wifi_run = true;
	uint old_main_cnt = shmPTR->main_cnt - 1;
	while (true) {
		if (shmPTR->wifi_run) {
			shmPTR->wifi_cnt++;
			if (shmPTR->main_cnt == old_main_cnt) {
				if (++errors >= 3) {
					flag = 1;
					cout << "main dont update cnt! EXIT\n";
					return;
				}
			}
			else {
				errors = 0;
				old_main_cnt = shmPTR->main_cnt;
			}
			delay(100);
			cnt_wifi_strong++;
			cnt_wifi_strong &= 31;
			if (cnt_wifi_strong == 1)
			{
				get_signal_strong();
			}
		}
		else {
			flag = 1;
			cout << "wifi: recived EXIT command\n";
			return;
		}
	}

}

std::ofstream out;
std::streambuf *coutbuf;// старый буфер

int main(int argc, char *argv[])
{
	init_shmPTR();

	int tmp = shmPTR->wifi_cnt;
	delay(1000);
	if (tmp != shmPTR->wifi_cnt) {
		cout << "wifi_clone\n";
		return -1;
	}
	
	thread tl(watch_d);
	tl.detach();
	delay(100);
	if (flag == 1)
		return -1;
	//if (shmPTR->run_main == false)
		//return 0;

	if (signal(SIGINT, handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}
	if (signal(SIGPIPE, pipe_handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}

	if (argc == 2) {
		out = std::ofstream(argv[1]); //откроем файл для вывод
		coutbuf = std::cout.rdbuf(); //запомним старый буфер
		std::cout.rdbuf(out.rdbuf()); //и теперь все будет в файл!
		std::cerr.rdbuf(out.rdbuf());
	}


	shmPTR->connected = 0;
	shmPTR->client_addr = 0;
	shmPTR->telemetry_buf_len = 0;
	shmPTR->commander_buf_len = 0;

	//usleep(1000000)
	if (flag == 0) {
		//while (flag == 0 && shmPTR->run_main) {
			server();
		//}
	}


	shmdt((void *)shmPTR);
	cout << "   wifi exits...\n";
	out.close();
    return 0;
}




