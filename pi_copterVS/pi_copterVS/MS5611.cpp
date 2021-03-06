
#include "MS5611.h"
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"
#include "Log.h"
#include "GPS.h"
#include "ssd1306.h"
#include "define.h"
#include "LogReader.h"

#define NORM_CT 10000



int open_i2c() {
	int fd4S;
	if ((fd4S = open("/dev/i2c-0", O_RDWR)) < 0) {
		cout << "Failed to open the bus.\n";
		return -1;
	}

	if (ioctl(fd4S, I2C_SLAVE, MS5611_ADDRESS) < 0) {
		cout << "Failed to acquire bus access and/or talk to slave.\n";
		return -1;
	}
	return fd4S;

}



unsigned int PROM_read(const int DA, char PROM_CMD)
{
	uint16_t ret = 0;
	uint8_t r8b[] = { 0, 0 };

	if (write(DA, &PROM_CMD, 1) != 1){
		cout << "read set reg Failed to write to the i2c bus." << "\t"<<millis_() << endl;
		return 0;
	}

	if (read(DA, r8b, 2) != 2){
		cout << "Failed to read from the i2c bus." << "\t"<<millis_() << endl;
		return 0;
	}

	ret = r8b[0] * 256 + r8b[1];

	return ret;
}


char RESET = 0x1E;

//int MS5611Class::error(int e) {	return -1;}


int MS5611Class::writeReg(const int fd4S,char bar_zero) {
	if (write(fd4S, &bar_zero, 1) != 1) {
		cout << "write reset 8 bit Failed to write to the i2c bus." << "\t"<<millis_() << endl;
		bar_task = 0;
		return -1;
	}
	return 0;
}

double MS5611Class::alt() {
	return altitude_;
}
#ifndef WORK_WITH_WIFI
int cntssdde = 0;
#endif
double fspeed = 0;

#define MAX_D_PRESSURE 300.0

bool MS5611Class::fault() {
	return wrong_altitude_cnt > MAX_BAROMETR_ERRORS;
}

#define ALT_NOT_SET -10000
double MS5611Class::getAltitude(const int32_t i_pressure) {
	const double pressure = i_pressure;
	static double old_alt= ALT_NOT_SET;
	static double gps_barometr_alt_dif = ALT_NOT_SET;
	static uint init_cnt = 0;

#define gps_alt (GPS.loc.altitude - gps_barometr_alt_dif)
	
	double alt = (44330.0 * (1.0 - pow(pressure / PRESSURE_AT_0, 0.1902949)));
	if (old_alt == ALT_NOT_SET)
		old_alt = alt;

	if (fault()) {
		powerK = 1;
		return gps_alt - GPS_ALT_MAX_ERROR;
	}
	else {
		

		if (init_cnt++ > 20 && GPS.loc.accuracy_ver_pos_ <= 10) {

			if (gps_barometr_alt_dif == ALT_NOT_SET)
				gps_barometr_alt_dif = GPS.loc.altitude - altitude_;
			
			if (fabs(old_alt - alt) > 20 ) {
				alt = gps_alt;
				Telemetry.addMessage(e_BAROMETR_ERROR);
			}
			else
				gps_barometr_alt_dif += (GPS.loc.altitude - alt - gps_barometr_alt_dif) * 0.01;
			old_alt = alt;
		}

		powerK += (constrain(PRESSURE_AT_0 / pressure, 1, 1.2) - powerK)*0.001;
	}
	
	return alt;
}


void MS5611Class::log_sens() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::MS5611_SENS);// (LOG::MPU_SENS);
		Log.loadByte(i_readTemperature);
		Log.loadFloat((float)pressure);
		//Log.loaduint32t(P);
		Log.block_end();
	}
}

//--------------------------------------------------
#ifdef FLY_EMULATOR

#include "GPS.h"


#include "Balance.h"


int32_t ttimet = 0;
bool MS5611Class::loop(){
	if (millis_() - ttimet < 50)
		return false;


//	const double dt = 0.05;// (millis_() - timet)*0.001;
//	ttimet = millis_();
	const double new_altitude = Emu.get_alt();

	//speed = (new_altitude - altitude_) / dt;
	shmPTR->altitude_ = altitude_ = new_altitude;
	shmPTR->pressure = pressure = get_pressure(altitude_);


	i_readTemperature = 20;

	//speed = Emu.get_speedZ();
#ifdef Z_SAFE_AREA
	if (Autopilot.motors_is_on() && (altitude_ - altitude_error) > Z_SAFE_AREA) {
		Autopilot.control_falling(i_CONTROL_FALL);
	}
#endif




	
	log_sens();

	powerK = 101325.0 / (double)pressure;

	if (powerK>1.4)
		powerK = 1.4;
	if (powerK < 1)
		powerK = 1;
	return true;
}

#else



bool MS5611Class::loop(){
	bool ret = false;
	switch (bar_task)
	{
	case 0:
		ret = phase0();
		break;
	case 1:
		ret = phase1();
		break;
	default:
		ret = phase2();
	}
	//Mpu.ms5611_timed = Mpu.timed;
	return ret;
}

#endif

//----------------------------------------------------


void MS5611Class::error(const int n) {
	Telemetry.addMessage(e_BAROMETR_RW_ERROR);
	mega_i2c.beep_code(B_I2C_ERR);
	bar_task = 0;
	cout << "Failed to read-write from the i2c barometr bus #" <<n<<" "<< millis_() << endl;
	wrong_altitude_cnt++;
	ct = NORM_CT + NORM_CT;
}

bool MS5611Class::phase0() {
#ifndef LOG_READER
	bar_D[0] = bar_D[1] = bar_D[2] = 0;
	const int fd4S = open_i2c();
	if (fd4S == -1 || writeReg(fd4S, CONV_D2_4096) == -1) {
		close(fd4S);
		error(11);
		return true;
	}
	close(fd4S);
#endif
	b_timeDelay = micros_() + ct;
	bar_task = 1;
	return true;

}
#define fc C
bool MS5611Class::phase1(){

#ifdef LOG_READER
	if (micros_() > b_timeDelay) {
		b_timeDelay = micros_() + ct;
		bar_task = 2;
		bar_zero = 0;
		return true;
	}
	else
		return false;

#else

	if (micros_()  > b_timeDelay) {
		const int fd4S = open_i2c();
		if (fd4S == -1 || writeReg(fd4S,bar_zero) == -1) {
			error(21);
			close(fd4S);
			return true;
		}

		bar_task = 2;
		bar_zero = 0;
		
		bar_h = read(fd4S, &bar_D, 3);
		if (bar_h != 3) {
			error(22);
			close(fd4S);
			return true;
		}
		
		D2 = ((int32_t)bar_D[0] << 16) | ((int32_t)bar_D[1] << 8) | bar_D[2];
		int32_t dT = D2 - (uint32_t)fc[4] * 256;
		int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;
		int32_t TEMP2 = 0;
		if (compensation)
		{
			if (TEMP < 2000)
			{
				TEMP2 = (dT * dT) / (2 << 30);
			}
		}
		TEMP = TEMP - TEMP2;
		i_readTemperature = ((int8_t)(TEMP * 0.01));

		if (writeReg(fd4S, CONV_D1_4096) == -1) {
			error(23);
			close(fd4S);
			return true;
		}
		close(fd4S);
		b_timeDelay = micros_() + ct;
		return true;
	}
	else
		return false;
#endif
}

bool MS5611Class::phase2() {

	if (micros_()  > b_timeDelay)
	{
		bar_task = 0;
		bar_zero = 0;

#ifdef LOG_READER
		P = logR.sd.pressure;
		i_readTemperature = logR.sd.i_readTemperature;
#else
		const int fd4S = open_i2c();
		if (fd4S == -1 || writeReg(fd4S, bar_zero) == -1) {
			error(31);
			close(fd4S);
			return true;
		}
		
		bar_h = read(fd4S, &bar_D, 3);
		close(fd4S);
		if (bar_h != 3) {
			error(32);
			return true;
		}

		D1 = ((int32_t)bar_D[0] << 16) | ((int32_t)bar_D[1] << 8) | bar_D[2];
		int32_t dT = D2 - (uint32_t)fc[4] * 256;
		int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
		int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

		if (compensation)
		{
			int32_t TEMP = 2000 + ((int64_t)dT * fc[5]) / 8388608;
			int64_t OFF2 = 0;
			int64_t SENS2 = 0;
			if (TEMP < 2000)
			{
				OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
				SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
			}
			if (TEMP < -1500)
			{
				OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
				SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
			}

			OFF = OFF - OFF2;
			SENS = SENS - SENS2;
		}





		//double pr = (D1 * SENS / 2097152 - OFF) * 0.000030517578125;

		int32_t tP = ((((int64_t)D1*SENS) / 2097152 - OFF) / 32768);

		if (tP < 80000 || tP > 107000) {
			cout << "PRESSURE ERROR " << tP << "\t"<<millis_() << endl;
			error(33);
			return true;
		}
		else {
			wrong_altitude_cnt = 0;
			P = tP;
			ct = NORM_CT;
		}

#endif


		if (altitude_ == ALT_NOT_SET) {
			pressure = P;
			altitude_ =  getAltitude(pressure);	
		}
		pressure = P;
		log_sens();
		altitude_ = getAltitude(pressure);
		shmPTR->altitude_ = (int32_t)(altitude_ * 1000.0);
		shmPTR->pressure = pressure;

#ifdef Z_SAFE_AREA
		if (Autopilot.motors_is_on() && (altitude_ - altitude_error) > Z_SAFE_AREA) {
			Autopilot.control_falling(e_OUT_OF_PER_V);
		}
#endif

		return true;
	}
	else
		return false;

}

void MS5611Class::update(){}


double MS5611Class::get_pressure(double h) {
	return PRESSURE_AT_0 * pow(1 - h*2.25577e-5, 5.25588);
}

//-------------------------------------------init-------------------------
bool MS5611Class::init() {
	oldAltt = 100000;
	bar_task = 0;
	bar_zero = 0x0;
	ct = NORM_CT;


	wrong_altitude_cnt = 0;
	altitude_ = ALT_NOT_SET;
	//altitude_error = ALT_NOT_SET;

	powerK = 1;

#ifndef FLY_EMULATOR

	cout << "Initialize High resolution: MS5611\n";


#endif
	pressure = PRESSURE_AT_0;
	ms5611_count = 0;
#ifndef LOG_READER
#ifndef FLY_EMULATOR
	compensation = true;

	const int fd4S = open_i2c();


	if (fd4S == -1 || writeReg(fd4S,RESET) == -1) {
		cout << "Failed to writeReg(RESET)\n";
		myDisplay.textDisplay("MS5611 ERROR\n");
		return false;
	}

	usleep(100000);
	for (i = 0; i < 6; i++) {
		C[i] = PROM_read(fd4S, 0xA2 + (i * 2));
		if (C[i] == 0) {
			cout << "Failed PROM_read\n";
			myDisplay.textDisplay("MS5611 ERROR\n");
			return false;
		}
	}
	close(fd4S);

#endif
#endif
	return true;
}
MS5611Class MS5611;
