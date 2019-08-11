
#include "MS5611.h"
#include "Telemetry.h"
#include "Autopilot.h"
#include "debug.h"
#include "Log.h"
#include "GPS.h"


#define NORM_CT 10000

static int fd4S;
unsigned int PROM_read(int DA, char PROM_CMD)
{
	uint16_t ret = 0;
	uint8_t r8b[] = { 0, 0 };

	if (write(DA, &PROM_CMD, 1) != 1){
		cout << "read set reg Failed to write to the i2c bus.\n";
	}

	if (read(DA, r8b, 2) != 2){
		cout << "Failed to read from the i2c bus.\n";
	}

	ret = r8b[0] * 256 + r8b[1];

	return ret;
}


char RESET = 0x1E;
#define ALT_NOT_SET 0



int MS5611Class::error(int e) {
	return -1;
}


int MS5611Class::writeReg(char bar_zero) {
	if (write(fd4S, &bar_zero, 1) != 1) {
		cout << "write reset 8 bit Failed to write to the i2c bus.\n";
		bar_task = 0;
		return error(2);
	}
}


float MS5611Class::altitude() {
	return altitude_ - altitude_error;
}
#ifndef WORK_WITH_WIFI
int cntssdde = 0;
#endif
float fspeed = 0;
void MS5611Class::copterStarted(){
//	if (altitude_error == ALT_NOT_SET){
		altitude_error = altitude_;
//	}
}



#define MAX_D_PRESSURE 300.0

bool MS5611Class::fault() {
	return wrong_altitude_cnt > MAX_BAROMETR_ERRORS;
}
double MS5611Class::getAltitude(const double pressure) {
	double alt = (44330.0f * (1.0f - pow(pressure / PRESSURE_AT_0, 0.1902949f)));//Где блядь проверка 4.4.2018-вот она
	
	if (fault()) {
			return GPS.loc.altitude - gps_barometr_alt_dif - GPS_ALT_MAX_ERROR;
	}
	else {
		shmPTR->altitude_ = (int32_t)(alt*1000.0);
		shmPTR->pressure = pressure;
		gps_barometr_alt_dif += (GPS.loc.altitude - alt - gps_barometr_alt_dif)*0.1;
	}
	
	return alt;
}


void MS5611Class::log_sens() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::MS5611_SENS);// (LOG::MPU_SENS);
		Log.loadByte(i_readTemperature);
		Log.loadFloat(pressure);
		//Log.loaduint32t(P);
		Log.block_end();
	}
}

//--------------------------------------------------
#ifdef FALSE_WIRE

#include "GPS.h"


#include "Balance.h"


long ttimet = millis();
uint8_t MS5611Class::loop(){
	if (millis() - ttimet < 200)
		return 0;


	const float dt = 0.2;// (millis() - timet)*0.001;
	ttimet = millis();
		const float new_altitude = Emu.get_alt();

	speed = (new_altitude - altitude_) / dt;
	shmPTR->altitude = altitude_ = new_altitude;
	shmPTR->pressure = pressure = get_pressure(altitude_);


	i_readTemperature = 20;

	//speed = Emu.get_speedZ();
#ifdef Z_SAFE_AREA
	if (Autopilot.motors_is_on() && (altitude_ - altitude_error) > Z_SAFE_AREA) {
		Autopilot.control_falling(i_CONTROL_FALL);
	}
#endif




	
	log_sens();

	powerK = PRESSURE_AT_0 / pressure;

	if (powerK>1.4)
		powerK = 1.4;

}

#else

uint8_t MS5611Class::loop(){
	
	switch (bar_task)
	{
	case 0:
		phase0();
		break;
	case 1:
		phase1();
		break;
	default:
		phase2();
	}

	return 0;
}

#endif

//----------------------------------------------------




void MS5611Class::phase0() {
	bar_D[0] = bar_D[1] = bar_D[2] = 0;

	writeReg(CONV_D1_4096);
	b_timeDelay = micros() + ct;
	bar_task = 1;
}
#define fc C
void MS5611Class::phase1()
{
	if (micros()  > b_timeDelay) {
		bar_task = 2;
		bar_zero = 0;
		writeReg(bar_zero);
		bar_h = read(fd4S, &bar_D, 3);
		if (bar_h != 3) {
			cout << "Failed to read from the i2c bus " << bar_h << endl;
			error(3);
			return;
		}
		D1 = ((int32_t)bar_D[0] << 16) | ((int32_t)bar_D[1] << 8) | bar_D[2];
		writeReg(CONV_D2_4096);
		/*
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
		writeReg(CONV_D1_4096);
		*/
		b_timeDelay = micros() + ct;
	}
}

void MS5611Class::phase2() {    
	if (micros()  > b_timeDelay)
	{
		bar_task = 0;
		//openDev();
		bar_zero = 0;
		writeReg(bar_zero);

		bar_h = read(fd4S, &bar_D, 3);

		if (bar_h != 3)
			cout << "Failed to read from the i2c bus "<< bar_h << endl;


		D2 = ((int32_t)bar_D[0] << 16) | ((int32_t)bar_D[1] << 8) | bar_D[2];




		dT = D2 - (uint32_t)C[5] * 256;
		TEMP = (2000 + ((int64_t)dT * (int64_t)C[5] / 8388608));
		OFF = (int64_t)C[2] * 65536 + (dT*C[4]) / 128;
		SENS = (int32_t)C[1] * 32768 + dT * C[3] / 256;
		/*
		SECOND ORDER TEMPARATURE COMPENSATION
		*/
		if (TEMP < 2000) // if temperature lower than 20 Celsius 
		{
			int32_t T1 = 0;
			int64_t OFF1 = 0;
			int64_t SENS1 = 0;

			T1 = pow((double)dT, 2) / 2147483648;

			OFF1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
			SENS1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;


			if (TEMP < -1500) // if temperature lower than -15 Celsius 
			{
				OFF1 = OFF1 + 7 * ((TEMP + 1500) * (TEMP + 1500));
				SENS1 = SENS1 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
			}

			TEMP -= T1;
			OFF -= OFF1;
			SENS -= SENS1;
		}
		P = ((((int64_t)D1*SENS) / 2097152 - OFF) / 32768);




		/*

		
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





		//float pr = (float)(D1 * SENS / 2097152 - OFF) * 0.000030517578125;

		int32_t tP = ((((int64_t)D1*SENS) / 2097152 - OFF) / 32768);
		if (tP < 80000 || tP > 107000) {
			cout << "PRESSURE ERROR " << tP << endl;
			ct = NORM_CT +NORM_CT;
			wrong_altitude_cnt++;
		}
		else {
			wrong_altitude_cnt = 0;
			P = tP;
			ct = NORM_CT;
		}


		*/



		const double dt = 0.02;// (Mpu.timed - old_timed);
		old_timed = Mpu.timed;

		static float low_alt=0, low_alt1=0, low_alt2=0;

		if (pressure == PRESSURE_AT_0) {
			pressure = P;
			altitude_ = low_alt= low_alt1 = low_alt2 = getAltitude(pressure);
		}

		pressure += ((double)P - pressure)*0.3;
		log_sens();
		powerK = constrain(PRESSURE_AT_0 / pressure, 1, 1.2);
		const double new_altitude = getAltitude(pressure);
		speed = (new_altitude - altitude_) / dt;
		//Debug.load(0, speed / 10, dt);
		//Debug.dump();
		altitude_ = new_altitude;

		//////////////////
		low_alt += (altitude_ - low_alt)*0.007;
		acc = (low_alt - (2 * low_alt1) + low_alt2) / (dt*dt);
		low_alt2 = low_alt1;
		low_alt1 = low_alt;
		////////////////////


#ifdef Z_SAFE_AREA
		if (Autopilot.motors_is_on() && (altitude_ - altitude_error) > Z_SAFE_AREA) {
			Autopilot.control_falling(e_OUT_OF_PER_V);
		}
#endif

		
	}
}

void MS5611Class::update(){}


float MS5611Class::get_pressure(float h) {
	return PRESSURE_AT_0 * pow(1 - h*2.25577e-5, 5.25588);
}

//-------------------------------------------init-------------------------
int MS5611Class::init() {
	acc = 0;
	oldAltt = 100000;
	gps_barometr_alt_dif = 0;
	old_timed = 0;
	bar_task = 0;
	bar_zero = 0x0;
	ct = NORM_CT;
	

	wrong_altitude_cnt = 0;
	speed = altitude_ = 0;
	altitude_error = ALT_NOT_SET;

	powerK = 1;

#ifndef FALSE_WIRE

	cout << "Initialize High resolution: MS5611\n";


#endif
	pressure = PRESSURE_AT_0;
	ms5611_count = 0;

#ifndef FALSE_WIRE
	compensation = true;

	if ((fd4S = open("/dev/i2c-0", O_RDWR)) < 0) {
		cout << "Failed to open the bus.\n";
		return error(0);
	}

	if (ioctl(fd4S, I2C_SLAVE, MS5611_ADDRESS) < 0) {
		cout << "Failed to acquire bus access and/or talk to slave.\n";
		return error(1);
	}

	writeReg(RESET);

	usleep(100000);
	for (i = 0; i < 6; i++) {
		usleep(1000);

		C[i] = PROM_read(fd4S, CMD_PROM_READ + (i * 2));
		//fprintf(Debug.out_stream,"C[%d] = %d\n", i, C[i]);

	}

#endif
	return 0;
}
MS5611Class MS5611;
