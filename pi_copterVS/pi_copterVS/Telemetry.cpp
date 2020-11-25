// 
// 
// 
/*

4.20 В —— 100%
3.95 В —— 75%
3.85 В —— 50%
3.73 В —— 25%
3.50 В —— 5%
2.75 В —— 0%


*/

#include "Telemetry.h"
#include "GPS.h"
#include "Hmc.h"
#include "mpu.h"
#include "MS5611.h"
#include "commander.h"
#include "Balance.h"
#include "Stabilization.h"

#include "debug.h"
#include "mpu_umulator.h"
#include "Log.h"
#include "ssd1306.h"


#ifdef LOG_READER
#include "LogReader.h"
#endif






#define BALANCE_DELAY 120
#define MAX_FLY_TIME 1800
#define BAT_ZERO 300.0f
#define BAT_50P 350.0f
#define BAT__timeout 50
#define BAT_timeoutRep  2
//#define BAT_100P 422
#define MAX_UPD_COUNTER 100
#define MAX_VOLTAGE_AT_START 406

#define BATTERY_CAPASITY 10;

static float BAT_Ampere_hour = BATTERY_CAPASITY;
static float  f_current = 0;
static float fly_time_lef = 0;

static int old_voltage;



void TelemetryClass::addMessage(const string msg, bool and2sms){

	cout << msg << "\t"<<millis_() << endl;;
	if (message.length() < msg.length() || message.compare(msg) == -1) {
		message += msg;
	}

}

void TelemetryClass::getSettings(int n){

	if (n > 7 || n < 0)
		return;
	cout << "up set: "<<n<<"\n";
	ostringstream convert;
	convert << "UPS" << n <<",";
	message += convert.str();

	switch (n){
	case 0:
		message += Balance.get_set();
		break;
	case 1:
		message += Stabilization.get_z_set();
		break;
	case 2:
		message += Stabilization.get_xy_set();
		break;
	case 3:
		message += Autopilot.get_set(false);
		break;
	case 4:
		message += Mpu.get_set();
		break;
	case 5:
		message += Hmc.get_set();
		break;		
	case 6:
		message += Commander.get_set();
		break;
	case 7:
		message += Debug.get_set();
		break;
	}
	
/*	uint8_t o = (message.length()+1) % 3;  //чтобы длина сообщения была кратная трем.
	if (o == 1){
		message += "  ";
	}
	else if (o==2)
		message += " ";*/
	message += ",";
	//cout<<message<<endl;

}
static float full_battery_charge;
static float bat_chargedK;

float TelemetryClass::get_bat_capacity() {
	return BAT_Ampere_hour;
}

void TelemetryClass::set_bat_capacity(float a_ch) {
	BAT_Ampere_hour = a_ch;
	float lost = full_battery_charge - battery_charge;

	full_battery_charge = battery_charge = BAT_Ampere_hour * bat_chargedK;
	battery_charge -= lost;
}

#define voltage_fn "/home/igor/copter_work_info"






void TelemetryClass::save_voltage() {
	FILE* f = fopen(voltage_fn, "w");
	if (f!=NULL) {
		
		fprintf(f, "%i,%i,%i", (int)voltage, on_power_time, total_time);
		fclose(f);
	}
}

int TelemetryClass::get_saved_voltage() {
	char data[80];


	FILE* set = fopen(voltage_fn, "r");
	if (set) {
		fscanf(set, "%i,%i,%i", &old_voltage, &on_power_time, &total_time);
		fclose(set);
			
		return 0;
			
		
	}
	return -1;
}


void TelemetryClass::init_()
{

	get_saved_voltage();

	BAT_Ampere_hour = BATTERY_CAPASITY;
	init_shmPTR();

	buf = shmPTR->telemetry_buf;
	powerK = 1;
	minimumTelemetry = false;
	lov_voltage_cnt = 0;

	low_voltage = voltage50P=false;
	message = "";
	next_battery_test_time = BAT__timeout;
	update_voltage();
	SN = ((voltage < 1200) ? 3 : 4);
	newGPSData = false;
	//Out.println("TELEMETRY INIT");
	voltage_at_start = 0;
	full_power = 0;


	bat_chargedK = (voltage/SN - 322)*0.01;
	if (bat_chargedK > 1)
		bat_chargedK = 1;
	full_battery_charge=battery_charge = BAT_Ampere_hour * bat_chargedK;

	if (voltage - old_voltage > 5) {
		total_time += on_power_time / 1000;
		on_power_time = 0;
	}

	cout << "telemetry init OK \n";
}

uint16_t data[5];

bool TelemetryClass::loop()
{
	bool ret = false;
	const int32_t _ct = millis_();
	if (next_battery_test_time<_ct){
		next_battery_test_time = _ct + BAT__timeout;
		ret = true;
		testBatteryVoltage();
		uint16_t time_left = check_time_left_if_go_to_home();
		if (Autopilot.progState() && time_left < 60 && ++no_time_cnt>3){ 
			cout << "too far from HOME!" << "\t"<<millis_() << endl;
			addMessage(e_BATERY_OFF_GO_2_HOME);
			Autopilot.going2HomeStartStop(false);
		}	

		static uint8_t log_cnt = 0;
		if (Log.writeTelemetry) {
			Log.block_start(LOG::TELE);
			Log.loadMem((uint8_t*)data, 10, false);
			if (++log_cnt > 50) {
				Log.loadInt16t(time_left);
				//cout << time_left << endl;
			}
			Log.block_end();
		}
		
	}
	
	if (update_buf()) {
		message = "";

	}

	return ret;
}

int TelemetryClass::get_voltage4one_cell() { return (int)(voltage / SN); }
int TelemetryClass::fly_time_left() {
	return fly_time_lef;
}
int TelemetryClass::check_time_left_if_go_to_home(){
	float need_time=0;
	if (Autopilot.motors_is_on()) {
		need_time = 5.0f + GPS.loc.dist2home / Stabilization.get_def_max_speedXY();
		if (Mpu.get_Est_Alt() < HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME) 
			need_time += 5.0f+(HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME- Mpu.get_Est_Alt()) / Stabilization.get_max_speedZ_P_4_go_to_home();
		need_time += 5.0f+fabs(Mpu.get_Est_Alt() / Stabilization.get_max_speedZ_M_4_go_to_home());
	}
	return fly_time_lef-need_time;
}


#ifdef FLY_EMULATOR
#define FULL_FW
#endif

int TelemetryClass::update_voltage() {

#ifdef FULL_FW
	Emu.battery(m_current, voltage);
#else

#ifdef LOG_READER
	int ret = logR.parser(TELE);
	if (ret)
		memcpy((uint8_t*)data, (uint8_t*)logR.sd.data, 10);

#else
	if (mega_i2c.getiiiiv((char*)data) == -1) {
		cout << "Failed getiiiiv\n";
		return -1;
	}
#endif

	//const float hall_effect_sensor_max_cur[4] = { 36,37,43,34 };
	const float hall_effect_sensor_max_cur[4] = { 39,38,36,46 };
	static float data_at_zero[4] = { 3255,3255,3255,3255 };
	static float cur_k[4] = { 0.011,0.011,0.011,0.011 };
	if (Autopilot.motors_onState() == false) {
		for (int i = 0; i < 4; i++) {
			data_at_zero[i] += (((float)data[i]) - data_at_zero[i]) * 0.01f;
			cur_k[i] = hall_effect_sensor_max_cur[i] / data_at_zero[i];
		}
	}

	shmPTR->m_current[0] = m_current[0] = abs(hall_effect_sensor_max_cur[0] - ((float)data[0]) * cur_k[0]);
	shmPTR->m_current[1] = m_current[1] = abs(hall_effect_sensor_max_cur[1] - ((float)data[1]) * cur_k[1]);
	shmPTR->m_current[2] = m_current[2] = abs(hall_effect_sensor_max_cur[2] - ((float)data[2]) * cur_k[2]);
	shmPTR->m_current[3] = m_current[3] = abs(hall_effect_sensor_max_cur[3] - ((float)data[3]) * cur_k[3]);
	shmPTR->voltage = voltage = 0.564 * (float)(data[4]);
	full_power = (m_current[0] + m_current[1] + m_current[2] + m_current[3]) * voltage;


	/*
   //Debug.dump((float)data[0], (float)data[1], (float)data[2], (float)data[3]);
   static float maxi[4] = { 0,0,0,0 };
   int i = 3 & (int)Debug.sett[0];
   maxi[i] += (m_current[i]-maxi[i])*0.01;
   Debug.dump(maxi[i],i, 0, data[i]);
	   */

	   //Debug.dump((motor>=0)?m_current[motor]:-1, powerI-restI,0,0);
	//if (m_current[0] > 1  || m_current[1] > 1 || m_current[2] > 1 || m_current[3] > 1 )
	 //  Debug.dump(m_current[0], m_current[1], m_current[2], m_current[3]);
	//   Debug.dump(m_current[0]+ m_current[1]+ m_current[2]+ m_current[3],0,0,0);

#endif

	return 0;
}


bool TelemetryClass::testBatteryVoltage() {
	static int32_t old_time = millis_();
	if (update_voltage() == -1) {
		myDisplay.textDisplay("BATTERY ERROR\n");
		return false;
	}
	//const double time_nowd = Mpu.timed;
	const int32_t _ct = millis_();

	double dt = 0.001 * (_ct - old_time);

	old_time = _ct;
	if (dt > 0.5)
		dt = 0.5;
	float current = (m_current[0] + m_current[1] + m_current[2] + m_current[3]-0.09 );

	f_current += (current - f_current) * 0.03;


	battery_charge -= (current * dt / 3600);
	if (battery_charge < 0)
		battery_charge = 0;


	if (Autopilot.time_at__start) {
		float fly_time = 0.001 * (millis_() - Autopilot.time_at__start);
		if (fly_time > 15)
			fly_time_lef = fly_time / (1 - (battery_charge / full_battery_charge)) - fly_time;
		else
			fly_time_lef = MAX_FLY_TIME - fly_time;
		//cout << predict_fly_time << endl;
	}
	//printf("charge=%f, cons ch=%f, bat ch=%f\n", current,consumed_charge, battery_charge);

	if (!Autopilot.motors_is_on())
		shmPTR->voltage_at_start = voltage_at_start = voltage;
	

	if (voltage < BAT_ZERO*SN)
		lov_voltage_cnt++;
	else
		lov_voltage_cnt = 0;

	low_voltage = lov_voltage_cnt > 3;
	voltage50P = voltage < BAT_50P*SN;

	//if (low_voltage)
	//	addMessage(e_VOLT_MON_ERROR);

	float t_powerK = (MAX_VOLTAGE_AT_START * SN) / voltage;
	powerK += (constrain(t_powerK, 1, 1.35f) - powerK)*0.001;
	return true;
}

bool newGPSData = false;




void TelemetryClass::loadBUF32(int &i,  int32_t val)
{
	buf[i++] = ((byte*)&val)[0];
	buf[i++] = ((byte*)&val)[1];
	buf[i++] = ((byte*)&val)[2];
	buf[i++] = ((byte*)&val)[3];

}

void TelemetryClass::loadBUF16(int &i, int16_t val)
{
	buf[i++] = ((byte*)&val)[0];
	buf[i++] = ((byte*)&val)[1];


}



void TelemetryClass::loadBUF8(int &i,  const float val){
	int8_t t = (int8_t)(val);
	buf[i++] = ((byte*)&t)[0];
}
void TelemetryClass::loadBUF(int &i, const float fval)
{
	int16_t t = (int16_t)(fval);
	buf[i++] = ((byte*)&t)[0];
	buf[i++] = ((byte*)&t)[1];
	 
}


 /////////////////////////////////////////////////////////////////////////////////////////////////
 //uint8_t telemetry_cnt = 0;

bool gps_or_acuracy = false;



int32_t last_update_time=0;

bool TelemetryClass::update_buf() {
	const int32_t ct = millis_();
	if (shmPTR->connected == 0 || shmPTR->telemetry_buf_len > 0) // || ct-last_update_time<20)
		return false;
	if (Autopilot.busy()) {
		shmPTR->telemetry_buf_len=4;
		return false;
	}
	last_update_time = ct;
	//bzero(buf, 32);
	//delay(1000);
	int i = 0;
	uint32_t mod = Autopilot.get_control_bits();

	if (shmPTR->status & 0x8000)
		mod |= WIFI_CAMERA_FOUND;
	if (shmPTR->inet_ok)
		mod |= INET_OK;

//	printf("out <- %i\n", mod);
	loadBUF32(i, mod);
	//printf("message=", message.c_str());
	loadBUF(i, 1000 + (Balance.get_throttle() * 1000));
	loadBUF32(i, GPS.loc.lat_);
	loadBUF32(i, GPS.loc.lon_);

	buf[i++] = (byte)GPS.loc.accuracy_hor_pos_;
	buf[i++] = (byte)GPS.loc.accuracy_ver_pos_;

	loadBUF(i, 10.0f*Autopilot.corectedAltitude4tel());// -Autopilot.startAltitude));
	//Out.printf(t_old_alt); Out.printf(" "); Out.println(MS5611.altitude);// -Autopilot.startAltitude);
	loadBUF8(i, -Mpu.get_pitch());
	loadBUF8(i, Mpu.get_roll());
	loadBUF8(i, Balance.c_pitch);
	loadBUF8(i, Balance.c_roll);

	loadBUF8(i, fmin((GPS.loc.speed_hor * 3), 127));
	loadBUF8(i, fmin((Mpu.get_Est_SpeedZ() * 5), 127));
	int16_t d2h = GPS.loc.dist2home;
	loadBUF16(i, (d2h <= 0x7fff) ? d2h : 0x7fff - d2h);
	loadBUF16(i, (int)(voltage*10*4/SN));



	//----
	loadBUF16(i, battery_charge*1000);
	loadBUF16(i, f_current *1000);
	loadBUF16(i, Mpu.vibration * 1000);

	buf[i++] = (int8_t)Autopilot.getGimbalPitch();

	


	float yaw=Mpu.get_yaw();
	loadBUF16(i, (int16_t)(yaw * 182.0));



	loadBUF32(i, on_power_time+Autopilot.powerOnTime());
	loadBUF32(i, shmPTR->status);
	loadBUF8(i, cpu_temp);

	if (message.length() && i + message.length() + 2 < TELEMETRY_BUF_SIZE) {
		loadBUF16(i, message.length());
		memcpy(&buf[i], message.c_str(), message.length());
		i += message.length();
	}
	else {
		buf[i++] = 0;
		buf[i++] = 0;
	}

	uint8_t *b;
	int len;
	do {
		b = Log.getNext(len);
		if (len == 0) {
			buf[i++] = 0;
			buf[i++] = 0;
			break;
		}
		if (i + len + 6 < TELEMETRY_BUF_SIZE) {
			loadBUF16(i, len);
			memcpy(&buf[i], b, len);
			i += len;
		}
		else
			break;

	} while (true);
	shmPTR->telemetry_buf_len = i;
	static int cnttt = 0;
	cnttt++;
//	Debug.dump((float)cnttt, 0, 0, 0);
	return true;
}
//nado echo peredat koordinaty starta i visoti ili luche ih androis socharanaet

TelemetryClass Telemetry;

