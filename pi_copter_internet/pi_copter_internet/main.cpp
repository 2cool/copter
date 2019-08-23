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
using namespace std;


void telegram_send_video_frame(string name);

//#define FORTEST

#ifdef FORTEST
//#include "../../../../2coolz/pi_copter_mega/pi_copterVS/pi_copterVS/glob_header.h"
#include "C:/Users/Igor/pi_copter_mega/pi_copterVS/pi_copterVS/glob_header.h"

#else
#include "../pi_copterVS/glob_header.h"
#endif





//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}
void pipe_handler(int sig) {
	cout << "wifi pipe error\n";
}





#define PPP_INET
#define TELEGRAM_BOT_RUN
#define LOGER_RUN

bool f_start_loger=false, f_start_telegram = false, f_start_ppp=false;
bool telegram_inet_ok = false, loger_inet_ok = false;

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
	//cout << "close pipe\n";
	pclose(pipe);
	return result;
}











string sos_msg;

key_t          ShmKEY;
int            ShmID;
struct Memory *shmPTR;

int init_shmPTR() {
	if (shmPTR == 0) {


		ShmKEY = ftok(SHMKEY, 'x');
		ShmID = shmget(ShmKEY, sizeof(struct Memory),  0666);
		if (ShmID < 0) {
			cout << "*** shmget error (internet) ***\n";
			return 1;
		}
		shmPTR = (struct Memory *) shmat(ShmID, NULL, 0);
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////
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
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))


//////////////////////////////////////////////////////////////////////////

/*
https://api.telegram.org/bot272046998:AAESv6nbLLWWm1nGaYPRc9Etr04XhY3aUww/getUpdates
https://api.telegram.org/bot272046998:AAESv6nbLLWWm1nGaYPRc9Etr04XhY3aUww/sendMessage?chat_id=241349746&text=hi_or

*/


static unsigned int telegram_cnt = 0;

string sms_phone_number;
string sms_mes;

static const std::string head = "curl -k -s \"https://api.telegram.org/bot272046998:AAESv6nbLLWWm1nGaYPRc9Etr04XhY3aUww/";
static const std::string htext = "curl -k -s \"https://api.telegram.org/bot272046998:AAESv6nbLLWWm1nGaYPRc9Etr04XhY3aUww/sendMessage?chat_id=241349746&text=";

volatile int _sms_n;


inline bool space(char c) {
	return std::isspace(c);
}

inline bool notspace(char c) {
	return !std::isspace(c);
}

string mes2send;
string telNumber;
void sendsms() {

	/*
	gammu sendsms TEXT 601xxx555 -text "To is TESTING. A SMS that is!"
	*/

	string com = "gammu sendsms TEXT " + sms_phone_number + " -text \"" + mes2send + "\"";
	string ret=exec(com);

}




//-----------------------------------------------
string getLocation() {
	std::string req = "lat:" + std::to_string(shmPTR->lat_) + " lon:" + std::to_string(shmPTR->lon_) + " alt:" + std::to_string((int)(shmPTR->gps_altitude_/1000)) + " hor:" + std::to_string((int)shmPTR->accuracy_hor_pos_);
	return req;
}


void add_stat(string &send) {
	if (shmPTR->control_bits & MOTORS_ON){
		send += "m_on,";
		if (shmPTR->control_bits & GO2HOME)
			send += "go2home,";
		else if (shmPTR->control_bits & PROGRAM) 
			send += "prog,";
	}
	else
		send += "m_off,";
	send += "b" + std::to_string((int)shmPTR->voltage) + ",";
	send += getLocation() + ",";

}
string down_case(string &str) {
	for (uint i = 0; i < str.length(); i++) {
		if (str[i] >= 65 && str[i] <= 90)
			str[i] += 32;
	}
	return str;
}
const static int  com_bit[] = { MOTORS_ON ,GO2HOME,CONTROL_FALLING,REBOOT,SHUTDOWN,GIMBAL_PLUS,GIMBAL_MINUS,PROGRAM,Z_STAB,XY_STAB,NOT_USED1,NOT_USED2,MPU_GYRO_CALIBR,COMPASS_CALIBR };
const static string str_com[] = { "motorson","go2home","cntrf","reboot","shutdown","gimbp","gimbm","prog","zstab","xystab","compason","horizonton","mpugyrocalibr","compasscalibr","stat", "help","image" };
const static int arr_size = sizeof(com_bit) / 4;
//-----------------------------------------------------------------------------
void parse_messages_(string message, string &send) {

	message = down_case(message);
	const string in = send;
	const bool seccure_f = true;// (message.find("282496") != string::npos);
	if (seccure_f) {

		for (int i = 0; i < arr_size; i++) {

			if (message.find(str_com[i]) != string::npos) {
				shmPTR->control_bits_4_do = com_bit[i];
				send += str_com[i];
				break;
			}

		}
		if (message.find(str_com[14]) != string::npos) {//stat
			add_stat(send);
		}
		else if (message.find(str_com[15]) != string::npos) {//help
			for (int i = 0; i < 17; i++)
				send += str_com[i] + ",";
		}
		else if (message.find(str_com[16]) != string::npos) {
			if (f_start_telegram) {
				
				string name = exec("date +%s");// +".jpg";
				name = name.substr(0, name.length() - 1) + ".jpg";
				telegram_send_video_frame(name);
			}
		}
	}
	if (send.compare(in) == 0) {
		send = "";

	}
	cout << "recived mess: " << message << endl;

}

//-----------------------------------------------------------------------------
void parse_sms_command() {

	mes2send = "";
	parse_messages_(sms_mes, mes2send);
	if (mes2send.length()>0)
		sendsms();
	sms_phone_number = "";
	sms_mes = "";
	
}


/*

apt install gammu
sudo pico /etc/gammurc
insert just this:

[gammu]
device = /dev/ttyS2
connection = at115200

root@2cool:~# gammu —getsms 0 2
Location 2, folder "Inbox", SIM memory, Inbox folder
SMS message
SMSC number          : "+38050000402"
Sent                 : Wed 23 May 2018 11:37:10 AM  +0300
Coding               : Unicode (no compression)
Remote number        : "Vodafone"
Status               : Read
User Data Header     : Concatenated (linked) message, ID (8 bit) 157, part 2 of 3

іх умов тарифу поповніть рахунок від 40 гривень. Деталі: www.vodafo


root@2cool:~# gammu —getsms 0 4
Location 4, folder "Inbox", SIM memory, Inbox folder
SMS message
SMSC number          : "+380672021111"
Sent                 : Wed 23 May 2018 12:02:12 PM  +0300
Coding               : Default GSM alphabet (no compression)
Remote number        : "+380973807646"
Status               : Read

Stat


=============================



0 SMS parts in 0 SMS sequences
root@2cool:~/projects/pi_copter_internet# gammu getallsms
Location 1, folder "Inbox", SIM memory, Inbox folder
SMS message
SMSC number          : "+38050000401"
Sent                 : Thu 24 May 2018 12:50:13 PM  +0300
Coding               : Unicode (no compression)
Remote number        : "Internet"
Status               : UnRead
User Data Header     : Concatenated (linked) message, ID (8 bit) 222, part 1 of 2

ttttt Увага! ПрАТ «ВФ Україна» не несе відповідальності за текст

Location 2, folder "Inbox", SIM memory, Inbox folder
SMS message
SMSC number          : "+38050000401"
Sent                 : Thu 24 May 2018 12:50:18 PM  +0300
Coding               : Unicode (no compression)
Remote number        : "Internet"
Status               : UnRead
User Data Header     : Concatenated (linked) message, ID (8 bit) 222, part 2 of 2

повідомлення.


gammu deletesms 0 2 2


apt-get install ppp
/etc/ppp/peers/vodafon115200

connect "/usr/sbin/chat -v -f /etc/chatscripts/gprs -T Bearer"
/dev/ttyS2
115200
noipdefault
usepeerdns
defaultroute
persist
noauth
nocrtscts
local

*/

int readsms_n() {
	


	string str = "gammu getsms 0 " +to_string(_sms_n);
	str = exec(str);
	if (str.length() < 20) {
		return -1;
	}
	string tttt;
	//int smsc_number =	str.find("SMSC number          : \"");
	std::size_t  remote_number =  str.find("Remote number        : \"");
	std::size_t status =		  str.find("Status               : UnRead");
	if (status != string::npos) {
			
		if (remote_number != string::npos) {
			remote_number += 24;
			std::size_t len = str.substr(remote_number).find("\"");
			if (len != string::npos) {
				sms_phone_number = str.substr(remote_number, len);
				if (sms_phone_number.find("+380") == string::npos)
					sms_phone_number = "+380973807646";



			}
			else
				return -1;
		}
		else
			return -1;

		std::size_t mess = str.find("\n\n");
		if (mess != string::npos && mess > status) {
			std::size_t len = str.substr(mess + 2).find("\n\n");
			if (len != string::npos) {
				sms_mes = str.substr(mess + 2, len);
			}
			else
				return -1;
		}
		else
			return -1;

			parse_sms_command();
		}
		string com = "gammu deletesms 0 " + to_string(_sms_n);
		system(com.c_str());
		
		
	return 0;

}

void readsms() {
	if (_sms_n > 0)
		readsms_n();
	else {//read all

		for (int i = 1; i <= 20; i++) {
			_sms_n = i;
			if (readsms_n() == -1)
				break;
		}
	}
}
void sendSMS(string message) {

	mes2send = message;
	shmPTR->sms_at_work = 2;

}


void send_sos(string msg) {
	sos_msg = msg;
	mes2send = msg + ":";
	add_stat(mes2send);
	sms_phone_number = "+380973807646";
	sendsms();
	//..............
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////

void _stop_ppp_read_sms_start_ppp() { // external
	if (f_start_ppp) {
		shmPTR->ppp_run = false;
		cout << "ppp_run stop\n";
		while (shmPTR->inet_ok) {
			delay(100);
		}

		cout << "inet off\n";
	}
	_sms_n = 0;
	shmPTR->sms_at_work = 1;

	while (shmPTR->sms_at_work) {
		delay(100);
	}
	cout << "sms_done\n";
	delay(100);
	shmPTR->ppp_run = f_start_ppp;
}




void sms_loop() {
	while (true) {
		while (shmPTR->sms_at_work == 0) {
			delay(100);
		}
		if (shmPTR->sms_at_work == 1) {
			if (f_start_ppp) {
				shmPTR->ppp_run = false;
				while (shmPTR->inet_ok)
					delay(100);
			}
			readsms();
		
		}
		if (shmPTR->sms_at_work == 2) {
			if (f_start_ppp) {
				shmPTR->ppp_run = false;
				while (shmPTR->inet_ok)
					delay(100);
			}
			sendsms();
		}

		shmPTR->sms_at_work = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////






/*

 ffmpeg -rtsp_transport udp -i "rtsp://192.168.42.1:554/live" -vframes 1 /home/igor/logs/image.jpg


curl -s -X POST "https://api.telegram.org/bot272046998:AAESv6nbLLWWm1nGaYPRc9Etr04XhY3aUww/sendPhoto" -F chat_id=241349746 -F photo="@/home/igor/logs/image.jpg"

*/






void telegram_send_video_frame(string name) {
	string ex = "ffmpeg -rtsp_transport udp -i rtsp://192.168.42.1:554/live -vframes 1 /home/igor/logs/" + name;
	exec(ex);
	cout << "save image " << ex << endl;
	ex = "curl -s -X POST https://api.telegram.org/bot272046998:AAESv6nbLLWWm1nGaYPRc9Etr04XhY3aUww/sendPhoto -F chat_id=241349746 -F photo=@/home/igor/logs/" + name;
	exec(ex);
	cout << "send image " << ex << endl;
}





void telegram_loop() {
	static uint old_message_len = 0;
	static int32_t last_update = 0;
	static int32_t last_message_t = millis();

	shmPTR->telegram_run = true;

	while (true) {


		while ( shmPTR->inet_ok == false || shmPTR->telegram_run == false) {
			delay(100);
			telegram_cnt = 0;
			telegram_inet_ok = false;
		}

		
			
		delay(100);
		int32_t time = millis();
		//commander
		const int32_t interval = ((time - last_message_t) < 20) ? 1000 : 10000;
		if (time - last_update > interval) {
			//printf("upd\n");
			last_update = time;
			std::string upd = "" + exec(head + "getUpdates\"");
			//printf("%s\n", upd.c_str());
			telegram_inet_ok=(upd.length() > 0 && upd.find("ok") >= 0);
			if (old_message_len == 0 || old_message_len > upd.length())
				old_message_len = upd.length();
			else
				if (old_message_len < upd.length()) {
					old_message_len = upd.length();
					last_message_t = time;
					//int dat_pos = 6 + upd.rfind("date\":");
					uint mes_pos = upd.rfind(",\"text\":\"");
					if (mes_pos != string::npos) {
						mes_pos += 9;
						const uint mes_end = upd.rfind("\"}}");
						if (mes_end != string::npos && mes_end > mes_pos) {
							std::string send = htext;
							std::string message = upd.substr(mes_pos, mes_end - mes_pos);

							parse_messages_(message, send);
							if (send.length() > 0) {
								send = exec(send + " \"");
								//raise(SIGPIPE);
							}
						}
					}
				}
			

		}
		if (telegram_cnt == 0) {
			std::string send = exec(htext + "bot started\"");
			cout << "telegram bot started\n";
		}
		telegram_cnt++;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------
/*
GT02 Communication Protocol
*/
unsigned short serial_n = 1;
static unsigned char com[42] = { 0x68,0x68, 0x25, 0,0, 0x08,0x66,0x26,0x20,0x31,0x36,0x98,0x45, 00,01, 0x10, 0x12, 0x04, 0x21, 0x9, 0x21, 0x00, 0,0,0,0, 0,0,0,0, 0,    0,0,   0,0,0, 0,0,0,0, 0x0D, 0x0A };
void getCommunication() {
	
	std::time_t t = std::time(0);
	std::tm* now = std::localtime(&t);
	/*std::cout << (now->tm_year + 1900) << '-'	<< (now->tm_mon + 1) << '-'	<< now->tm_mday<< " " <<now->tm_hour<<":"<<now->tm_min<<":"<<now->tm_sec<< "\n";*/
	unsigned  char tb[4];
	((unsigned short*)tb)[0] = serial_n++;
	int i = 14;
	com[i++] = tb[1];
	com[i++] = tb[0];

	com[i++] = now->tm_year - 100;
	com[i++] = now->tm_mon + 1;
	com[i++] = now->tm_mday;
	com[i++] = now->tm_hour;
	com[i++] = now->tm_min;
	com[i++] = now->tm_sec;


	((unsigned int*)tb)[0] = (unsigned int)((double)shmPTR->lat_*0.18);
	com[i++] = tb[3];
	com[i++] = tb[2];
	com[i++] = tb[1];
	com[i++] = tb[0];

	((unsigned int*)tb)[0] = (unsigned int)((double)shmPTR->lon_*0.18);

	com[i++] = tb[3];
	com[i++] = tb[2];
	com[i++] = tb[1];
	com[i++] = tb[0];


	com[i++] = (unsigned char)(3.6*sqrt(shmPTR->speedX*shmPTR->speedX + shmPTR->speedY*shmPTR->speedY));

	float course = shmPTR->yaw;
	if (course < 0)
		course = 360 + course;

	((unsigned short*)tb)[0] = course;

	com[i++] = tb[1];
	com[i++] = tb[0];

	i += 6;

	com[i] = 0x7;// device has been located, northern latitude, eastern longitude, not charged
}

//imei 866262031369845
/*
srv1.livegpstracks.com" или "5.9.136.109",
порт: 3335
*/
double distance_(const double lat, const double lon, const double lat2, const double lon2) {
	double R = 6371000;
	double f1 = (lat);
	double f2 = (lat2);
	double df = (lat2 - lat);
	double dq = (lon2 - lon);

	double a = (sin(df / 2) * sin(df / 2) + cos(f1) * cos(f2) * sin(dq / 2) * sin(dq / 2));
	return R * 2 * atan2(sqrt(a), sqrt(1 - a));

}


void loger_loop() {
	static double old_lat = 0, old_lon = 0;
	int sockfd_loger=0;
	shmPTR->loger_run = true;

	while (true) {
		delay(1000);
		double lat = 1.74532925199433e-9 * (double)shmPTR->lat_;  //radians
		double lon = 1.74532925199433e-9 * (double)shmPTR->lon_;
		//bearing = bearing_(lat, lon, lat + DELTA_A_RAD, lon + DELTA_A_RAD);
		const double distance = distance_(lat, lon, old_lat, old_lon);
		if (distance>10) {
			//cout << "sended" << endl;
			old_lat = lat;
			old_lon = lon;
			while (shmPTR->inet_ok == false || shmPTR->loger_run == false) {
				loger_inet_ok = false;
				delay(100);
				if (serial_n > 1) {
					cout << "loger stoped\n";
					if (sockfd_loger != 0) {
						close(sockfd_loger);
						sockfd_loger = 0;
					}
				}
				serial_n = 1;
			}
			if (serial_n <= 1)
				cout << "loger started\n";

			//	if (false)//GPS.loc.accuracy_hor_pos_>MIN_ACUR_HOR_POS_4_JAMM || abs(GPS.loc.dist2home_2 - last_dist2home2) < max(625, GPS.loc.accuracy_hor_pos_*GPS.loc.accuracy_hor_pos_))
			//		continue;

			if (serial_n == 1) {
				struct sockaddr_in serv_addr;
				struct hostent* server;
				int portno = 3335;
				sockfd_loger = socket(AF_INET, SOCK_STREAM, 0);

				if (sockfd_loger < 0) {
					//printf( "ERROR opening socket\n");
					serial_n = 1;
					continue;
				}

				server = gethostbyname("srv1.livegpstracks.com");
				if (server == NULL) {
					//printf( "ERROR, no such host\n");
					serial_n = 1;
					continue;
				}

				bzero((char*)& serv_addr, sizeof(serv_addr));
				serv_addr.sin_family = AF_INET;
				bcopy((char*)server->h_addr, (char*)& serv_addr.sin_addr.s_addr, server->h_length);
				serv_addr.sin_port = htons(portno);
				if (connect(sockfd_loger, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) < 0) {
					//printf( "ERROR connecting");
					serial_n = 1;
					continue;
				}
				//------------------------------------------------------
			}

			getCommunication();
			int n = write(sockfd_loger, com, 42);
			loger_inet_ok = n >= 42;
			if (n < 42) {
				cout << "loger: ERROR writing to socket\n";
				serial_n = 1;
				close(sockfd_loger);
				continue;
			}

		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int start_ppp() {
	if (shmPTR->inet_ok == true)
		return 0;
	delay(500);
	while (shmPTR->sim800_reset_time > 0)
		delay(100);


	shmPTR->ppp_run = true;
#ifdef PPP_INET
	cout << "starting ppp...\n";
	system("poff -a");
	delay(3000);
	system("pon  vodafon115200");
	delay(3000);
	system("route add default dev ppp0");
	delay(3000);
#endif
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////
static int no_inet_errors = 0;
void stop_ppp(bool test_inet=true) {
	if (!f_start_ppp)
		return;

	if (test_inet && shmPTR->inet_ok == false)
		return;
#ifdef PPP_INET
	system("route add default dev wlan0");
	delay(3000);
	system("poff -a");
	delay(3000);
	cout << "---------PPP STOPED---------\n";
#endif
	shmPTR->inet_ok = telegram_inet_ok = loger_inet_ok = false;
	delay(1000);
}


void test_ppp_inet_and_local_loop() {
	int n = 2;
	
	while (shmPTR->ppp_run && shmPTR->run_main) {
		delay(15000);
		if (!telegram_inet_ok && !loger_inet_ok) {
			string ret = exec("ping -w 5 -c 1 8.8.8.8");
			if (ret.find("1 received") == string::npos) {
				if (--n <= 0) {
					cout << "no inet " << ret << " " << n << endl;
					no_inet_errors++;
					break;
				}
			}
			else {
				if (shmPTR->inet_ok == false) {
					cout << "internet OK\n";
					shmPTR->inet_ok = true;
				}
				n = 2;
				no_inet_errors = 0;
				delay(3000);

			}
		}
		
	}
}
/////////////////////////////////////////////////////////////////////////////
void ppp_loop() {



	while (shmPTR->run_main && flag==0) {
		while (shmPTR->sms_at_work )
			delay(100);

		//stop_ppp();
		start_ppp();
		test_ppp_inet_and_local_loop();
		stop_ppp(false);
		if (no_inet_errors >= 2) {
			no_inet_errors = 0;
#ifdef PPP_INET
			system("poff -a");
			sleep(3);
			system("pkill gammu");
#endif
			shmPTR->sim800_reset = true;
		}



		while (shmPTR->ppp_run == false && shmPTR->run_main)
			delay(100);

	}
}

//////////////////////////////////////////////////////////////////////////////////////

void watch_d() {
	uint old_main_cnt= shmPTR->main_cnt-1;
	while (true) {
		if (shmPTR->internet_run) {
			shmPTR->internet_cnt++;
			if (shmPTR->main_cnt == old_main_cnt) {
				flag = 1;
				cout << "main dont update cnt! EXIT\n";
				return;
			}
			old_main_cnt = shmPTR->main_cnt;
			delay(100);
		}
		else {
			flag = 1;
			cout << "inet: recived EXIT command\n";
			return;
		}
	}
}


std::ofstream out;
std::streambuf *coutbuf;// старый буфер

int main(int argc, char *argv[])//lat,lon,.......
{
	printf("<start loger y> <start telegram y> <cout to file>\n");
	if (argc >= 3) {

		f_start_loger = (argv[1][0] == 'y' || argv[1][0] == 'Y');
		f_start_telegram = (argv[2][0] == 'y' || argv[2][0] == 'Y');
		f_start_ppp = f_start_loger | f_start_telegram;
	}
	if (argc >=4 ){
		out = std::ofstream(argv[3]); //откроем файл для вывод
		coutbuf = std::cout.rdbuf(); //запомним старый буфер
		std::cout.rdbuf(out.rdbuf()); //и теперь все будет в файл!
		std::cerr.rdbuf(out.rdbuf());
	}

	if (signal(SIGINT, handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}
	if (signal(SIGPIPE, pipe_handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}

	static bool stop_start = false;
	if (init_shmPTR())
		return 0;
	if (shmPTR->run_main == false)
		return 0;

//	if (shmPTR->internet_run)
//		return 0;

	int tmp = shmPTR->internet_cnt;
	delay(1000);
	if (tmp != shmPTR->internet_cnt) {
		cout << "inet_clone\n";
		return -1;
	}

	thread wd(watch_d);
	wd.detach();

	delay(400);
	if (flag)
		return 0;
#ifdef PPP_INET
	system("poff -a");
#endif
	sleep(3);
	system("gammu getallsms");
	system("gammu deleteallsms 1");

	shmPTR->inet_ok = false;
	shmPTR->internet_run = true; //main loop
	shmPTR->loger_run = false;
	shmPTR->telegram_run = true;
	shmPTR->sms_at_work = 0;


	


	cout << "start sms loop\n";
	thread tsms(sms_loop);
	tsms.detach();
	if (f_start_ppp) {
		//shmPTR->internet_run = true;
		cout << "start ppp loop\n";
		thread tppp(ppp_loop);
		tppp.detach();
	}
	if (f_start_loger) {
		cout << "start loger loop\n";
		thread tl(loger_loop);
		tl.detach();
	}
	if (f_start_telegram) {
		cout << "start telegram loop\n";
		thread tg(telegram_loop);
		tg.detach();
	}


	while ( shmPTR->run_main && flag==0) {
		if ( stop_start == false && shmPTR->stop_ppp_read_sms_start_ppp ) {
			stop_start = true;
			cout << "RING BIT recived:\n";
			_stop_ppp_read_sms_start_ppp();
			shmPTR->stop_ppp_read_sms_start_ppp = stop_start = false;
		}
		else {
			usleep(100000);
		}
	}
	shmPTR->ppp_run = false;
	
	stop_ppp();
	
	shmPTR->internet_run = false;
	
	cout << "internet's down...\n";
	shmdt((void *)shmPTR);

	out.close();
    return 0;
}