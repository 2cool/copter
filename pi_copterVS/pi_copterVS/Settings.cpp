// 
// 
// 

#include "Settings.h"
#include "Telemetry.h"
#include "Hmc.h"
#include "Balance.h"
#include "Stabilization.h"
#include "commander.h"
#include "ssd1306.h"
template <class T> int writeAnything(int ee, const T& value)
{
	const byte* p = (const byte*)(const void*)&value;
	unsigned int i;
	
	return i;
}

template <class T> int readAnything(int ee, T& value)
{
	byte* p = (byte*)(void*)&value;
	unsigned int i;
	
	return i;
}



uint32_t get_hash(const char *mem, const int len) {

	// Hashing
	uint32_t magic = 5381;
	for (int i = 4; i < len; i++)
		magic = ((magic << 5) + magic) + mem[i]; // magic * 33 + c		
	return magic;
}


void SettingsClass::init()
{


}


void SettingsClass::writeBuf(uint8_t adr, float buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		writeAnything(i * 4 + adr, buf[i]);
	}
}
void SettingsClass::readBuf(uint8_t adr, float buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		readAnything(i * 4 + adr, buf[i]);
	}
}
void SettingsClass::writeBuf(uint8_t adr, int16_t buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		writeAnything(i * 2 + adr, buf[i]);
	}
}
void SettingsClass::readBuf(uint8_t adr, int16_t buf[], uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		readAnything(i * 2 + adr, buf[i]);
	}
}

static float ar_t333[SETTINGS_ARRAY_SIZE + 1];
float * load(const string  buf, const uint8_t  * filds, const bool deny_zero) {


	ar_t333[SETTINGS_ARRAY_SIZE] = SETTINGS_IS_OK;
	for (int i = 0; i < SETTINGS_ARRAY_SIZE; i++) {
		string sval = buf.substr(filds[i], filds[i + 1] - filds[i] - 1);
		if (deny_zero && sval.length() == 1 && sval[0] == '0') {
			ar_t333[SETTINGS_ARRAY_SIZE] = SETTINGS_ERROR;
			cout << " !!!!! " << buf << endl;
			return ar_t333;
		}

		float val = (float)stod(sval);

		ar_t333[i] = val;

	}

	return ar_t333;
}






bool SettingsClass::load_(string buf, bool any_ch) {


	any_change = any_ch;

	cout << "settings\n";
	uint8_t filds[11];
	uint8_t fi = 0;
	uint8_t i = 0;

	i++;

	while (fi < 10 && i < buf.length()) {
		while (buf[i++] != ',');
		filds[fi++] = i++;

	}
	filds[10] = (uint8_t)buf.length();

	uint8_t n = (uint8_t)stoi(buf.substr(0, filds[0] - 1));
	switch (n) {
	case 0:
		Balance.set(load(buf, filds,true));
		break;
	case 1:
		Stabilization.setZ(load(buf, filds,true));
		break;
	case 2:
		Stabilization.setXY(load(buf, filds,true));
		break;
	case 3:
		Autopilot.set(load(buf, filds,false));//secure
		break;
	case 4:
		Mpu.set(load(buf, filds,true));
		break;
	case 5:
		Hmc.set(load(buf, filds,false));
		break;
	case 6:
		Commander.set(load(buf, filds,false));
		break;
	case 7:
		Debug.set(load(buf, filds, false));
		break;
	default:
		return false;
	}
	return true;

}




#define MAX_CHANGE 0.2
void SettingsClass::set(const float  val_, float &set) {
	if (any_change) {
		set = val_;
	}

	bool neg = false;
	float val = val_;
	if (set < 0 && val < 0) {
		neg = true;
		set = -set;
		val = -val;

	}

	if (set > 0 && val > 0) {
		if (val > set) {
			if (val > set + set * MAX_CHANGE)
				set += set * MAX_CHANGE;
			else
				set = val;

		}
		else if (val < set) {
			if (val < set - set * MAX_CHANGE)
				set -= set * MAX_CHANGE;
			else
				set = val;
		}
		if (neg) {
			set = -set;
		}
	}

}

int SettingsClass::read_all() {

#ifndef FLY_EMULATOR



	FILE *f = fopen("/home/igor/copter_set.txt", "r");
	if (f == NULL)
	{
		cout << "No settings file!\n";
		myDisplay.textDisplay("NO SETTINGS FILE!\N");
		
		return -1;
	}
	
	char buf[1000];
////	char c=fgetc(f);
//	cout << c << endl;
	while (true)  {
		char *ret=fgets(buf, 1000, f);
		if (ret == NULL)
			break;
		
		load_(string(buf),true);
	}
	fclose(f);
	Hmc.calibration(false);

#endif
	return 0;
}
//replace 0, to 0.0, end add 1s to end and endl;
string r0(string str) {
	string nstr =  (str[0] == 0 && str[1] == ',')? "0.0":str.substr(0,1);
	uint i = 1;
	for (; i < str.length()-1; i++) {
		if (str.at(i - 1) == ',' && str.at(i) == '0' && str.at(i + 1) == ',')
			nstr += "0.0";
		else
			nstr += str.substr(i, 1);
	}
	if (str.at(i - 1) == ',' && str.at(i) == '0')
		nstr += "0.0";
	else
		nstr += str.substr(i, 1);
	nstr += ",1,1,1,1,1,1,1,1,1,1\n";
	
	return nstr;
}
int SettingsClass::write_all() {
#ifndef FLY_EMULATOR

	Telemetry.save_voltage();

	FILE *f = fopen("/home/igor/copter_set.txt", "w");
	if (f == NULL)
	{
		cout << "Can't writing to settings file!\n";
		return -1;
	}
	//HMC

	//0 - error ; 0.0 is ok

	fprintf(f, "0,%s", (r0(Balance.get_set())).c_str());
	fprintf(f, "1,%s", (r0(Stabilization.get_z_set())).c_str());
	fprintf(f, "2,%s", (r0(Stabilization.get_xy_set())).c_str());
	fprintf(f, "3,%s", (r0(Autopilot.get_set(true))).c_str());
	fprintf(f, "4,%s", (r0(Mpu.get_set())).c_str());
	fprintf(f, "5,%s", (r0(Hmc.get_set())).c_str());
	fprintf(f, "6,%s", (r0(Commander.get_set())).c_str());

	//fprintf(f, "4,%s", (Mpu.get_set() + end).c_str());
	//fprintf(f, "5,%s", (Hmc.get_set() + end).c_str());
	//fprintf(f, "6,%s", (Commander.get_set() + end).c_str());
	//(f, "9,%s", (Hmc.get_calibr_set() + end).c_str());
	
	fclose(f);
#endif
	return 0;


}

int SettingsClass::read_commpas_callibration(const int index, int16_t sh[]) {
	cout << "load compass calibr: ";
	ifstream source;
	string fn = "/home/igor/hmc_calibration_";
	fn += to_string(index);
	fn += ".txt";
	source.open(fn.c_str(), fstream::in);
	if (source.is_open()) {
		cout << "#"<<index<<":\n";
		for (int i = 0; i < 6; i++) {
			source >> sh[i];
			cout << sh[i] << ",";
		}
		cout << endl;
		return 0;
	}
	cout << "FAILL!!!\n";
	return -1;
}
int SettingsClass::read_commpas_motors_correction(float sh[]) {
	cout << "load compass motor correcton: ";
	ifstream source;
	source.open("/home/igor/hmc_motors_correction.txt", fstream::in);
	if (source.is_open()) {
		cout << "compas mot:\n";
		for (int i = 0; i < 12; i++) {
			source >> sh[i];
			cout << sh[i] << ",";
		}
		cout << "OK\n";
		return 0;
	}
	cout << "FAILL!!!\n";
	return -1;
}

int  SettingsClass::write_commpas_motors_correction(const float sh[]) {
#ifndef FLY_EMULATOR
	cout<< "write compass motors correction ";
	ofstream f;
	f.open("/home/igor/hmc_motors_correction.txt", fstream::out | fstream::trunc);
	if (f.is_open()) {
		for (int i = 0; i < 12; i++) {
			f << to_string(sh[i]) << endl;
		}
	//	float dd[12];
	//	read_commpas_motors_correction(dd);
		cout << "OK\n";
		return 0;
	}
	cout << "FAILED!!!\n";
	return -1;
#else
	return 0;
#endif
}

int SettingsClass::write_commpas_callibration(const int index, const int16_t sh[]) {
#ifndef FLY_EMULATOR
	cout << "write compass calibr is ";
	ofstream f;
	string fn= "/home/igor/hmc_calibration_";
	fn += to_string(index);
	fn += ".txt";
	f.open(fn.c_str(), fstream::out | fstream::trunc);
	if (f.is_open()) {
		for (int i = 0; i < 6; i++) {
			f << to_string(sh[i]) << endl;
		}
		int16_t dd[6];
		read_commpas_callibration(index, dd);
		cout << "OK/n";
		return 0;
	}
	cout << "FAILED!!!/n";
	return -1;
#else
	return 0;
#endif;

}

/////////////////////////////////////



SettingsClass Settings;

