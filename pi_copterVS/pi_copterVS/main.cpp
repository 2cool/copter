
 
#define PROG_VERSION "ver: 3.201014\n"
#define SIM800_F


//Additional Options  DDEBUG

//при стартре замерять вибрацию после чего делать корекцию или вообще запрещать полет при сильной вибрации

#include <sys/sem.h>
#include  <sys/types.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>

#include <unistd.h>
#include <sys/shm.h>
#include <sys/ipc.h>

#include <cstdio>
#include <signal.h>
#include  <stdio.h>
#include  <stdlib.h>
#include <sys/types.h>


#include "ssd1306.h"

#include "define.h"

#include "Log.h"

#include "debug.h"



#include "WProgram.h"/// not change
#include "Settings.h"
#include "Prog.h"
#include "Location.h"

#include "GPS.h"
#include "Telemetry.h"
#include "commander.h"


#include "Autopilot.h"

#include "mi2c.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Balance.h"


bool loop();



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.


// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

//#include <avr/io.h> //  can be omitted
//#include <avr/interrupt.h> // can be omitted


int zzz = 1;



void set_wifi_30_() {
	exec("sudo ifconfig wlan0 down");
	delay(3000);
	exec("sudo iw reg set BO");
	delay(3000);
	exec("sudo iwconfig wlan0 txpower 30");
	delay(3000);
	exec("sudo ifconfig wlan0 up");
	delay(2000);
//	exec("iwconfig");
	//delay(5000);
}



#define WIND_X 7
#define WIND_Y 0
#define WIND_Z 0

int init(int cnt) {////--------------------------------------------- INITIALIZATION ------------------------------
	Log.init(cnt);
#ifdef FLY_EMULATOR
	Emu.init(WIND_X, WIND_Y, WIND_Z);
#endif
	bool ok = true;
	cout << "___setup___\n";
	ok &= GPS.init();
	ok &= Commander.init();
	ok &= Mpu.init();
	ok &= Hmc.init();
	Balance.init();
	ok &= MS5611.init();
	Autopilot.init();
	Telemetry.init_();
	ok &= Telemetry.testBatteryVoltage();

	Settings.read_all();
	ok &= Hmc.calibrated_;
	if (!Hmc.calibrated_)
		myDisplay.textDisplay("HMC NOT CALIBR\n");

	for (int i = 0; i < 3; i++) {
		MS5611.loop();
		usleep(20000);
	}

	cout << "press "<<MS5611.pressure<<" alt " << MS5611.alt()<<" temp "<<(int)MS5611.i_readTemperature << endl;
	return ok?0:-1;

}


uint8_t teil = 0, maxTeilN = 3;

#ifndef WORK_WITH_WIFI
bool foo_flag = false;
int32_t ttiimer = 0;
#endif


bool temp4test = true;


int ok_cnt = 0;
int ok_ccc = 0;
int er_cnt = 0;
long dt_sum=0;
int max_dt = 0;
int old_debug = 0;


uint64_t timer = 0;
bool loop()
{

	uint64_t beg = micros_();
	if (beg - timer < 9900)
	{
		usleep(9900 - (beg - timer));
	
	}

	timer = beg;

	Mpu.loop();

	if (!Hmc.loop())
		MS5611.loop();

	if (!Telemetry.loop())
		GPS.loop();
	Balance.loop();
	Commander.input();
	Autopilot.loop();

	if (shmPTR->sim800_reset_time > 0 && shmPTR->sim800_reset_time + 40e3 < millis_())
		shmPTR->sim800_reset_time = 0;

#ifdef FLY_EMULATOR
		usleep(3000);
#endif
		return true;
}

//http://www.cprogramming.com/tutorial/lesson10.html
volatile sig_atomic_t flag = 0;
void handler(int sig) { // can be called asynchronously
	flag = 1; // set flag
}
void pipe_handler(int sig) {
	cout << "pipe error" << "\t"<<millis_() << endl;
}


uint8_t get_cpu_temp() {

	int cpu_temp;
	FILE* cpuT = fopen("/etc/armbianmonitor/datasources/soctemp", "r");
	if (cpuT) {
		fscanf(cpuT, "%i", &cpu_temp);
		if (cpu_temp > 1000)
			cpu_temp /= 1000;
		return cpu_temp;
	}
	return 0;
}


int printHelp() {
	cout << PROG_VERSION << endl;
	cout << "<-help> for this help\n";
	cout << " <fly at start at hight in sm > <lower hight in sm> <f=write stdout to file > <log com and tel y> <start wifi>\n";
	cout << "example to write in log file : pi_copter 300 100 f n y \n";
	cout << "example to write in stdout   : pi_copter 300 100 s n y \n";
	return -1;
}
int32_t last_wifi__reloaded = 0, inet_start__cnt=0;
string stdout_file_ext = "";
//-----------------------------------------------------------------------------------------
bool start_wifi = false;// , start_inet = false, start_loger = false, start_telegram = false;;

bool max_cpu_freq = false;
bool min_cpu_freq = false;
bool network_manager_running = true;

void watch_dog() {
	delay(1000);
	while (shmPTR->run_main) {
		const uint8_t wifi_cnt = shmPTR->wifi_cnt;
		const uint8_t internet_cnt = shmPTR->internet_cnt;
		const uint8_t fpv_cnt = shmPTR->fpv_cnt;





		if (network_manager_running) {
#ifndef DEBUG
			static uint8_t ap_ok = 0;
			static uint8_t camera_ok = 0;
			const string cam =  exec("ifconfig wlx1cbfce0162cc | grep 192.168.42");
			camera_ok += cam.length() > 5;
			if (camera_ok == 1)
				myDisplay.textDisplay("cam-ok, ");
			

			const string wifi = exec("ifconfig wlx983f9f1908da | grep 192.168.1");
			ap_ok += wifi.length() > 5;
			if (ap_ok == 1)
				myDisplay.textDisplay("AP-ok, ");
			
			if (camera_ok && ap_ok) {
				string ret = exec("systemctl | grep NetworkManager.service");
				if (ret.length() > 10) {
					cout << "stop " << ret << endl;
					system("systemctl stop NetworkManager.service && \
							ifconfig wlx983f9f1908da down && \
							iw reg set GY && \
							ifconfig wlx983f9f1908da up && 	\
							iwconfig wlx983f9f1908da txpower 30 && \
							wpa_supplicant -B -iwlx983f9f1908da -Dnl80211 -c /etc/wifi.conf && \
							dhclient wlx983f9f1908da && \
							ifconfig wlx1cbfce0162cc up && \
							wpa_supplicant -B -iwlx1cbfce0162cc -c /etc/camera.conf -Dnl80211 && \
							dhclient wlx1cbfce0162cc");
					delay(3000);
				}
				network_manager_running = false;
			}
			
#else
			network_manager_running = false;
#endif // !DEBUG
		}
		delay(3000);

		Telemetry.cpu_temp=get_cpu_temp();
		if (!min_cpu_freq)
			if (Telemetry.cpu_temp >= 60 && Autopilot.motors_is_on()) {
				min_cpu_freq = true;
				system("nice -n -20 cpufreq-set -u 480000 -d 480000");
				cout << "temp to hight!!  set max freq to 480 mhz:\t" << millis_() << endl;
			}
			else {
				if (!max_cpu_freq) {
					max_cpu_freq = true;
					//cout << "set cpu freq to 1200 mhz:\t" << millis_() << endl;
					//system("nice -n -20 cpufreq-set -u 1370000 -d 1370000");
				}
				
			}
			
		if (Autopilot.busy())
			continue;

		const int32_t _ct = millis_();

		if (!network_manager_running) {
#define START_FPV 
#ifdef START_FPV
			if (fpv_cnt == shmPTR->fpv_cnt) {
				cout << "fpv killed\n";
				shmPTR->fpv_run = false;
				system("nice -n -20 pkill fpv_");
				delay(1000);
				shmPTR->fpv_run = true;
				cout << "fpv started\n";
				system("nice -n -20 /root/projects/fpv_ &"); // if write video to file  fpv_ f
			}
#else
			shmPTR->fpv_run = true;
#endif
			
#define START_FIFI
#ifdef START_FIFI
			if (start_wifi)
				if (wifi_cnt == shmPTR->wifi_cnt || (Autopilot.last_time_data__recived && (_ct - Autopilot.last_time_data__recived) > 60e3 && (_ct - last_wifi__reloaded) > 60e3)) {
					last_wifi__reloaded = _ct;
					cout << "--------------wifi killed:\t" << _ct << endl;
					shmPTR->wifi_run = false;
					system("nice -n -20 pkill wifi_p");
					delay(1000);
					shmPTR->wifi_run = true;
					cout << "--------------wifi started:\t" << _ct << endl;
					string t = "nice -n -20 /root/projects/wifi_p ";
					t += " &";
					system(t.c_str());
				}
#else
			shmPTR->wifi_run = true;
#endif
		}
//#define START_INET
#ifdef START_INET
		if (Commander.start_sim800_control)
			if (internet_cnt == shmPTR->internet_cnt) {
				cout << "--------------ppp starting" << "\t" << _ct << endl;
				shmPTR->internet_run = false;
				system("nice -n -20 pkill ppp_p");
				delay(1000);
				shmPTR->internet_run = true;
				string t = "nice -n -20 /root/projects/ppp_p ";
				t += ((Commander.ppp_inet) ? "y" : "n");
				t += " ";
				t += (Commander.telegram_bot ? "y" : "n");
				t += " ";
				if (stdout_file_ext.length()) 
					t += stdout_file_ext + "i" + to_string(inet_start__cnt++) + ".txt";
				t += " &";
				cout << t << endl;
				system(t.c_str());

			}
#else
		shmPTR->internet_run = true;
#endif

	}
}






bool is_clone() {
	uint8_t temp = shmPTR->main_cnt;

	usleep(51123);
	if (temp != shmPTR->main_cnt) {
		cout << "clone\n";
		return true;
	}
	return false;
}

#define LOG_COUNTER_NAME "/home/igor/logs/logCounter"



#define MIN_N_4_COUNTER 10000
int find_bigest() {
	FILE* in;
	char buff[512];

	if (!(in = popen("ls /home/igor/logs", "r"))) {
		return 1;
	}
	int counter = 0;
	while (fgets(buff, sizeof(buff), in) != NULL) {
		string s = string(buff);
		int b = s.find_first_of("0123456789");
		int e = s.find_first_of(".");
		if (b > 0 && e + 4 > b) {
			int cnt = stoi(s.substr(b, e));
			if (cnt > counter)
				counter = cnt;
		}
	}
	fclose(in);
	counter++;
	if (counter < MIN_N_4_COUNTER) {
		counter = MIN_N_4_COUNTER;
		const int dir_err = system("mkdir /home/igor/logs");
		if (-1 == dir_err)
		{
			counter = 0;
		}

	}
	return counter;
}

int test_4_counterFile() {
	int counter = 0;
	FILE* set = fopen(LOG_COUNTER_NAME, "r");
	if (set) {
		fscanf(set, "%i", &counter);
		fclose(set);
		usleep(500);
		remove(LOG_COUNTER_NAME);
		if (counter < MIN_N_4_COUNTER)
		{
			counter = find_bigest();
		}
	}
	else {
		return counter = find_bigest();
	}
	return counter;
}



std::ofstream out;
std::streambuf *coutbuf;// старый буфер

int main(int argc, char* argv[]) {
	//system("ifconfig wlan1 down");
	int counter = test_4_counterFile();
	if (counter == 0)
		return 0;

	FILE* set = fopen(LOG_COUNTER_NAME, "w+");
	if (set) {
		fprintf(set, "%i\n", counter + 1);
		fclose(set);
	}
	else
		return 0;

	if (init_shmPTR()) 
		return 0;
	if (is_clone())
		return 0;

	const double d_uptime = std::stod(exec("awk '{print $1}' /proc/uptime"));
	system("cpufreq-set -u 1370000 -d 1370000");
	system("systemctl stop  remote-fs.target graphical.target  sound.target cryptsetup.target multi-user.target systemd-tmpfiles-clean.timer  motd-news.timer fstrim.timer apt-daily.timer apt-daily-upgrade.timer ");

	


	shmPTR->in_fly = (shmPTR->control_bits&MOTORS_ON);
	shmPTR->wifi_cnt = 0;
	shmPTR->run_main = true;
	shmPTR->inet_ok = false;
	shmPTR->fpv_zoom = 1;
	shmPTR->fpv_adr = 0;
	shmPTR->fpv_port = 0;
	shmPTR->connected = 0;
	shmPTR->fly_at_start = 3;
	shmPTR->lowest_altitude_to_fly = 1.6f;

	
	string fname;
	
	Debug.n_debug = 0;

	myDisplay.setWordWrap(TRUE);
	myDisplay.setDisplayMode(SSD1306::Mode::SCROLL);

	if (argc >= 2) {
		int tt = string(argv[1]).compare("-help");
		if (tt == 0) {
			return printHelp();
		}
		if (argc >= 6) {
			int t = atoi(argv[1]);
			//t = constrain(t, 300, 300);/////
			shmPTR->fly_at_start = 0.01f*(float)t;

			t = atoi(argv[2]);
			shmPTR->lowest_altitude_to_fly = 0.01f*(float)t;
			if (shmPTR->lowest_altitude_to_fly > shmPTR->fly_at_start)
				shmPTR->lowest_altitude_to_fly = shmPTR->fly_at_start;

			
			if (argv[3][0] == 'f' || argv[3][0] == 'F') {
#ifdef DEBUG
				stdout_file_ext = "/home/igor/logs/log_out" + to_string(counter);
#else
				stdout_file_ext = "/home/igor/logs/log_out_real" + to_string(counter);
#endif
				fname = stdout_file_ext+".txt";
				out = std::ofstream(fname.c_str()); //откроем файл для вывод
				coutbuf = std::cout.rdbuf(); //запомним старый буфер
				std::cout.rdbuf(out.rdbuf()); //и теперь все будет в файл!
			}
#ifdef DEBUG
			cout << "___________!!!DEBUG!!!___________\n";

			myDisplay.textDisplay("!!!DEBUG!!!, ");
#endif
#ifdef FLY_EMULATOR
			cout << "___________!!!FLY_EMULATOR!!!___________\n";
			myDisplay.textDisplay("!!!FLY_EMULATOR!!!, ");
#endif



			cout << "UPTIME=" << d_uptime << endl;
			
			Log.writeTelemetry = (argv[4][0] == 'y' || argv[4][0] == 'Y');
			shmPTR->wifi_run = start_wifi = (argv[5][0] == 'y' || argv[5][0] == 'Y');
			//start_inet = (argv[6][0] == 'y' || argv[6][0] == 'Y');
			//start_inet |= start_loger=(argv[7][0] == 'y' || argv[7][0] == 'Y');
			//start_inet |= start_telegram=(argv[8][0] == 'y' || argv[8][0] == 'Y');
			shmPTR->internet_run = Commander.ppp_inet;
		}

	}
	else
		return printHelp();


	//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

	cout << PROG_VERSION << endl;
	myDisplay.textDisplay(PROG_VERSION);

	cout << argv[0] << "\n"<< argv[1] << " " << argv[2] << " " << argv[3] << " " << argv[4]  <<endl;
	
	if (signal(SIGINT, handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}
	if (signal(SIGPIPE, pipe_handler) == SIG_ERR) {
		return EXIT_FAILURE;
	}





#ifdef DEBUG
	mega_i2c.DO_SOUND = 0;
#else
	mega_i2c.DO_SOUND = 1;
#endif;

	mega_i2c.init();

	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		if (0 != exec("ifconfig wlx983f9f1908da").find("wlx983f9f1908da")) {
			cout << "WIFI NOT FOUND\n";
			myDisplay.textDisplay("WIFI NOT FOUND\n");
			return 0;
		}

		if (0 != exec("ifconfig wlx1cbfce0162cc").find("wlx1cbfce0162cc")) {
			cout << "CAM WIFI NOT FOUNE\n";
			myDisplay.textDisplay("CAM WIFI NOT FOUND\n");
			return 0;
		}
	
	if (init(counter) == 0) {
		shmPTR->reboot = 0;

	
		//string wifi = exec("ifconfig wlx983f9f1908da | grep 192.168.1");


		myDisplay.textDisplay("connecting to AP and camera");
		thread tl(watch_dog);
		tl.detach();

		while (shmPTR->run_main){
			if (loop()) {
				shmPTR->main_cnt++;
			}
			if (flag)
				shmPTR->run_main = false;
		}
	}
	else {
		cout << "INITIALIZATION ERROR!!!\n";
		myDisplay.textDisplay("INITIALIZATION ERROR!!!\n");
	}
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

	if (flag!=0)
		cout<< "\n main Signal caught!" << "\t"<<millis_() << endl;
	//WiFi.stopServer();
	Log.close();

	shmPTR->internet_run = false;
	shmPTR->wifi_run = false;
	shmPTR->fpv_run = false;

	sleep(3);

	if (shmPTR->run_main==false)
		cout<< "\n exit\n";

	
	if (shmPTR->reboot) {
		switch (shmPTR->reboot) {
		case 1:
			Settings.write_all();
			myDisplay.textDisplay("---reboot---\n");
#ifndef DEBUG
			system("reboot");
#endif
			break;
		case 2:
			Settings.write_all();
			myDisplay.textDisplay("---shutdown now---\n");
			system("shutdown now");
			break;

		}
	}
	
	//close_shmPTR();
	out.close();

#ifdef ONLY_ONE_RUN

	if (-1 == semctl(semid, 0, IPC_RMID, 0))
	{
		printf("Error delete!\n");
	}
#endif

	//system("cpufreq-set -u 1370000 -d 480000");
	myDisplay.textDisplay("---THE END---");
	return 0;

}

