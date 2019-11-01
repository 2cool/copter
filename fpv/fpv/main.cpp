#include <cstdio>
#include <string>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <string.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/time.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <math.h>
#include <ctime>
#include <thread>
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#include <cstdio>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <fstream>
#include <arpa/inet.h>
using namespace std;

//static const string wlan_fpv = "wlx8c18d960bdda";  //RTL8188EU
static const string wlan_fpv = "ra0";//MT7601U

std::string exec(const std::string cmd) {
	//printf(cmd.c_str());
	//printf("\n");
	char buffer[128];
	std::string result = "";
	FILE* pipe = popen(cmd.c_str(), "r");
	if (!pipe) {
		//throw std::runtime_error("popen() failed!");
		cout << "inet pipe brock\n";
		return "";
	}
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}
	pclose(pipe);
	return result;
}



//#define FORTEST
#ifdef FORTEST
#include "C:/Users/Igor/pi_copter_mega/pi_copterVS/pi_copterVS/glob_header.h"
//#include "C:/Users/2coolz/pi_copter_mega/pi_copterVS/pi_copterVS/glob_header.h"

#else
#include "../pi_copterVS/glob_header.h"
#endif

key_t          ShmKEY;
int            ShmID;
struct Memory *shmPTR;

int init_shmPTR() {
	if (shmPTR == 0) {


		ShmKEY = ftok(SHMKEY, 'x');
		ShmID = shmget(ShmKEY, sizeof(struct Memory), 0666);
		if (ShmID < 0) {
			cout << "*** shmget error (fpv) ***\n";
			return 1;
		}
		shmPTR = (struct Memory *) shmat(ShmID, NULL, 0);
	}
	return 0;
}




bool is_connected(void) { return shmPTR->connected > 0; }


//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}
void pipe_handler(int sig) {
	cout << "fpv pipe error\n";
}
#include <chrono>

static auto start = chrono::steady_clock::now();

int64_t micros() {

	auto end = chrono::steady_clock::now();
	return chrono::duration_cast<chrono::microseconds>(end - start).count();
}
int32_t millis() {
	auto end = chrono::steady_clock::now();
	return (int32_t)chrono::duration_cast<chrono::milliseconds>(end - start).count();
}


void delay(unsigned long t) {
	usleep(t * 1000);
}





string stoken;
struct sockaddr_in address;
int sock = 0, valread;
struct sockaddr_in serv_addr;


bool print_error_message = true;
 
int open_socket() {



	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		if (print_error_message) {
			print_error_message = false;
			cout << "\nFPV:  Socket creation error \n";
		}
		return -1;
	}else
		cout << "FPV: Socket creation OK \n";

	memset(&serv_addr, '0', sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(7878);

	// Convert IPv4 and IPv6 addresses from text to binary form 
	if (inet_pton(AF_INET, "192.168.42.1", &serv_addr.sin_addr) <= 0)
	{
		if (print_error_message) {
			print_error_message = false;
			cout << "\nFPV: Invalid address/ Address not supported \n";
		}
		return -1;
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		if (print_error_message) {
			print_error_message = false;
			cout << "\nFPV: Connection Failed \n";
		}
		return -1;
	}

	print_error_message = true;
	return 0;
}
char buffer[1024] = { 0 };

// 769 cam foto
// 513 video start
// 514 video stop

void send_msg(uint16_t msg) {
	
	const string tosend= "{\"msg_id\":"+to_string(msg)+",\"token\":" + stoken + "}";

	send(sock, tosend.c_str(), strlen(tosend.c_str()), 0);
	read(sock, buffer, 1024);
}

//      tosend = '{"msg_id":14,"token":%s,"type":"fast","param":"%s"}' %(self.token, self.ZoomLevelValue)

int set_zoom(int zoom) {
	cout << "zoom=" << zoom << endl;
	string tosend = "{\"msg_id\":14,\"token\":" + stoken + ",\"type\":\"fast\",\"param\":\"" + to_string(zoom) + "\"}";
	//cout << tosend << endl;
	send(sock, tosend.c_str(), strlen(tosend.c_str()), 0);
	read(sock, buffer, 1024);
	//cout << "Live webcam stream is now available.\n";

	return 0;
}
int connect_to_camera() {
	string tosend = "{\"msg_id\":257,\"token\":0}";

	int cnt = 0;
	do {
		send(sock, tosend.c_str(), strlen(tosend.c_str()), 0);
		valread = read(sock, buffer, 1024);
		string str = buffer;
		int rval_i = str.find("rval");
		if (rval_i >= 0) {
			//cout << str.c_str() << endl;
			int beg = 9 + str.find("\"param\": ");
			int len = str.substr(beg).find(" }");
			stoken = str.substr(beg, len);
			cout << "FPV:  Connection to camera OK\n";
			print_error_message = true;
			return 0;
		}
	} while (cnt++ < 3);

	if (print_error_message) {
		print_error_message = false;
		cout << "FPV: Connection to camera ERROR: " << buffer << endl;
	}
	return -1;

	
}
int camera_video_stream() {
	//-------------------------------------------------------
		string tosend = "{\"msg_id\":259,\"token\":" + stoken + ",\"param\":\"none_force\"}";
		//cout << tosend.c_str() << endl;
		send(sock, tosend.c_str(), strlen(tosend.c_str()), 0);
		read(sock, buffer, 1024);
		cout << "Live webcam stream is now available.\n";
		return 0;
		
}
/////////////////////////////////////////////////////////////////////////////////


string intIP2strIP(uint32_t ip) {

	string sip = to_string(ip & 255) + "." + to_string((ip >> 8) & 255) + "." + to_string((ip >> 16) & 255)+"."+ to_string(ip >> 24);
	return sip;
}
void stop_ffmpeg_stream() {
	system("pkill ffmpeg");
}
void start_ffmpeg_stream() {
	uint32_t ip = (shmPTR->client_addr & 0x00ffffff) | shmPTR->fpv_adr << 24;
	string s = "ffmpeg -rtsp_transport udp -i \"rtsp://192.168.42.1:554/live\" -c copy -f h264 udp://" + intIP2strIP(ip) + ":" + to_string(shmPTR->fpv_port) + " > /dev/null 2>&1  &";
	cout << "stream started" << endl;
	system(s.c_str());


	std::time_t result = std::time(nullptr);
	//std::cout << std::asctime(std::localtime(&result))
//		<< result << " seconds since the Epoch\n";


	string file_name = "/home/igor/stream_"+std::to_string(result);
	file_name += +".mp4";
	s = "ffmpeg -rtsp_transport udp -i \"rtsp://192.168.42.1:554/live\" -c copy -r 60 -y " + file_name + " > /dev/null 2>&1  &";
	cout << "stream to "+ file_name + " started" << endl;
	system(s.c_str());

	
}

//  https://github.com/lwfinger/rtl8188eu


//for MT7601U  https://forum.armbian.com/topic/1819-solved-orange-pi-pc-and-2-ralink-mt7601u-dongle-usb-id-148f7601/

/*
добавил сюда

/etc/network/interfaces

вот это что предотвртило автоматически подключатся 

auto wlx8c18d960bdda
allow-hotplug
iface wlx8c18d960bdda inet dhcp
wpa-ssid YDXJ_1234567
wpa-psk 1234567890



/etc/wpa_supplicant.conf

network={
	ssid="YDXJ_1234567"
	psk="1234567890"
}

sudo wpa_supplicant -B -ira0 -c /etc/wpa_supplicant.conf -Dwext
sudo dhclient ra0

*/


const static string connect2camera = "ifconfig "+wlan_fpv+" up && wpa_supplicant -B -i"+wlan_fpv+" -c /etc/camera.conf -Dwext && dhclient "+ wlan_fpv;



void sleep3s() {
	for (int i = 0; i < 15; i++) {
		delay(200);
		shmPTR->fpv_cnt++;
	}
}
bool fpv_stream = false;
int main_cnt_err = 0;
int main()
{
	//stop_ffmpeg_stream();
	init_shmPTR();

	if (!shmPTR->fpv_run) {
		cout << "FPV EXIT\n";
		return 0;
	}
	int clone=shmPTR->fpv_cnt;
	int old_main_cnt = shmPTR->main_cnt;
	delay(700);
	if (clone != shmPTR->fpv_cnt) {
		cout << "FPV CLONE\n";
		return 0;
	}
	if (old_main_cnt == shmPTR->main_cnt) {
		cout << "FPV:  MAIN dont run\n";
		return 0;
	}

	do {
		static bool print = true;
		string ret=exec("nice -n -20  nmcli dev wifi | grep YDXJ_1234567");
		if (ret.length() < 10) {
			if (print) {
				cout << "camera wifi not found...\n";
				print = false;
			}
			sleep3s();
		}
		else
			break;

	} while (shmPTR->fpv_run);
	cout << "camera found\n";

	string ret = exec("nice -n -20 ifconfig "+ wlan_fpv);
	if (ret.find("192.168.42.") == string::npos) {
		system(connect2camera.c_str());
		sleep3s();
	}
	
	while (open_socket() == -1) 
		sleep3s();
	
	while (connect_to_camera()==-1)
		sleep3s();
	int zoom=0;
	shmPTR->fpv_zoom = 1;


	while (shmPTR->fpv_run) {
		delay(200);
		shmPTR->fpv_cnt++;
		if (shmPTR->fpv_adr != 0 && shmPTR->fpv_port != 0) 
		{
			if (!fpv_stream) 
			{
				fpv_stream = true;
				camera_video_stream();
				start_ffmpeg_stream();
				//zoom = shmPTR->fpv_zoom;
				//set_zoom(zoom - 1);
				//cout << "start_fpv" << endl;
			}
		}
		else if (fpv_stream)
		{
			cout << "stop_fpv" << endl;
			fpv_stream = false;
			stop_ffmpeg_stream();
		}
		if (zoom != shmPTR->fpv_zoom) {
			zoom = shmPTR->fpv_zoom;
			//cout << "zoom="<< zoom << endl;
			set_zoom(zoom - 1);
		}
		if (shmPTR->fpv_code) {
			cout << "fpv code:"<<shmPTR->fpv_code << endl;
			send_msg(shmPTR->fpv_code);
			shmPTR->fpv_code = 0;
		}


		if (shmPTR->main_cnt == old_main_cnt) {
			if (++main_cnt_err > 5)
				break;
		}
		else
			main_cnt_err = 0;


		old_main_cnt = shmPTR->main_cnt;
	}

	stop_ffmpeg_stream();
	shutdown(sock, SHUT_RDWR);
	
	cout << "FPV EXIT\n";
  
    return 0;
}