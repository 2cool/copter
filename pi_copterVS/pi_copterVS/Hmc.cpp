// 
// 
// 

//#define SERIAL_PRINT

#include "Hmc.h"
#include "Telemetry.h"
#include "mpu.h"
#include "define.h"
#include "Autopilot.h"
#include "Settings.h"
#include "Balance.h"
#include "mpu.h"
#include "MS5611.h"
#include "debug.h"
#include "Log.h"


#define DEFAULT_DEV "/dev/i2c-0"



//enum{X,Y,Z};
void HmcClass::init()
{
	do_compass_motors_calibr = false;
	motor_index = 0;
	startTime = 10e3;

#ifdef COMPAS_MOTORS_OFF
	motors_power_on = false;
#else
	motors_power_on = true;
#endif

	c_base[X] = c_base[Y] = c_base[Z] = 0;
	dx = dy = dz = 0;
	//heading = 0;
	ok = true;
	calibrated = true;
#ifndef FLY_EMULATOR


	cout << "Initializing I2C devices...\n";
	initialize();
	delay(100);
	initialize();
	cout << "Testing device connections...\n";
	ok = testConnection();
	if (ok)
		cout << "HMC5883L connection successful\n";
	else
		cout << "HMC5883L connection failed\n";

	if (ok) {
		calibration(false);
	}

#endif	
}


string HmcClass::get_set(){


	ostringstream convert;
	convert << \
		(motors_power_on ? "1" : "0") << "," << \
		Mpu.yaw_correction_angle* RAD2GRAD;
	string ret = convert.str();
	return string(ret);

	//string s = motors_power_on ? "1" : "0";

//	//Out.printf("hmc set:"); Out.println(s);
	//return s;
}

void HmcClass::set(const float buf[]){
	motors_power_on = (buf[0] > 0);
	float yaw = constrain(buf[1],-45, 45);
	Mpu.yaw_correction_angle = yaw * GRAD2RAD;
	cout << "compas " << buf[0] << ","<<buf[1]<<endl;
}
//---------------------------------------------------------
void HmcClass::log_base() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::HMC_BASE);
		Log.loadMem((uint8_t*)c_base, 6, false);
		Log.block_end();
	}
}  
void HmcClass::log_sens() {
	if (Log.writeTelemetry) {
		Log.block_start(LOG::HMC_SENS);
		Log.loadMem(buffer, 6, false);
		Log.block_end();
	}
}




int baseI = 0;
bool motors_is_on_ = false;

void HmcClass::start_motor_compas_calibr(){
	if (do_compass_motors_calibr == false && Autopilot.motors_is_on()==false){
		cout << "START MOTOR COMPAS CAL\n";
		do_compass_motors_calibr = true;
		motors_is_on_ = false;
		c_base[X] = c_base[Y] = c_base[Z] = 0;
		startTime = millis_() + 5e3;
		motor_index = 0;

	}
}

#define MAX_M_WORK_VOLTAGE 1250.0f

float _base[3];
float current = 0;
void HmcClass::motTest(const float fmx, const float fmy, const float fmz){
	if (millis_() > startTime){
		if (baseI < 1000){
			if (Balance.gf(motor_index) == 0 || Balance.gf(motor_index) > MIN_THROTTLE) {
				current += Telemetry.get_current(motor_index);
				_base[0] += fmx;
				_base[1] += fmy;
				_base[2] += fmz;
				baseI++;
			}
		}
		else{
			if (motors_is_on_){

				//float fv = MAX_M_WORK_VOLTAGE / Telemetry.get_voltage();

				

				//fv*=(PRESSURE_AT_0/MS5611.pressure);
				Autopilot.motors_do_on(false, "CMT");
				motors_is_on_ = false;
				int index = motor_index * 3;

				printf(" MOTOR ON\n");
				printf("compas test: 4 m %i\n", motor_index);
				printf("%f\t%f\t%f\n", base[0], base[1], base[2]);
				printf("current 4 m %i %f\n", motor_index, Telemetry.get_current(motor_index)/1000);


				base[index] =   ((_base[0] - base[index])   / current);
				base[index+1] = ((_base[1] - base[index+1]) / current);
				base[index+2] = ((_base[2] - base[index+2]) / current);

							

				if (motor_index < 3){
					motor_index++;
					startTime = millis_() + 3e3;
				}
				else{
					do_compass_motors_calibr = false;
					Autopilot.reset_compas_motors_calibr_bit();
					Settings.saveCompasMotorSettings(base);
				}
			}
			else{
				int index = motor_index * 3;
				base[index] =   _base[0];
				base[index+1] = _base[1];
				base[index+2] = _base[2];
				printf(" MOTOR OFF\n");
				printf("compas test: 4 m %i\n",motor_index);
				printf("%f\t%f\t%f\n",base[0],base[1],base[2]);
				startTime = millis_() + 3e3;
				Autopilot.motors_do_on(true, "CMT");
				motors_is_on_ = true;
			}
			baseI = 0;
			_base[0] = _base[1] = _base[2] = current=0;
		}
	}
	
}

#ifdef FLY_EMULATOR


void HmcClass::loop(){
static int32_t comTime = 0;
///#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
	if (millis_() - comTime < 50)
		return;
	comTime = millis_();
	//heading = Emu.get_heading();
	
//	headingGrad = 0;


#ifndef WORK_WITH_WIFI
	headingGrad = 0;
#endif
	calibrated = true;
	//log();
}

#else

void HmcClass::loop(){

	static int hmc_cnt = 0;
	if (1 & hmc_cnt++)
		return;


	//double ttt = micros();
	//бельіе ноги
	int16_t mx,my,mz;
	
	if (readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer) == -1) {
		Telemetry.addMessage(e_COMMPAS_RW_ERROR);
		mega_i2c.beep_code(B_I2C_ERR);
		return;
	}
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mx = (((int16_t)buffer[0]) << 8) | buffer[1];
	my = (((int16_t)buffer[4]) << 8) | buffer[5];
	mz = (((int16_t)buffer[2]) << 8) | buffer[3];
	fmy = -(float)(mx - c_base[X])*dx;
	fmx = -(float)(my - c_base[Y])*dy;
	fmz = -(float)(mz - c_base[Z])*dz;

#ifndef SESOR_UPSIDE_DOWN
	my = -my;
	mz = -mz;
#endif

	if (do_compass_motors_calibr)
		motTest(fmx, fmy, fmz);

	if (Autopilot.motors_is_on() && motors_power_on){
		float kx, ky, kz,k;
		//m0;
			k = Telemetry.get_current(0);
			kx = base[0] * k;
			ky = base[1] * k;
			kz = base[2] * k;

		//m1;
			k = Telemetry.get_current(1);
			kx += base[3] * k;
			ky += base[4] * k;
			kz += base[5] * k;

		//m2
			k = Telemetry.get_current(2);
			kx += base[6] * k;
			ky += base[7] * k;
			kz += base[8] * k;

		//m3
			k = Telemetry.get_current(3);
			kx += base[9]  * k;
			ky += base[10] * k;
			kz += base[11] * k;
	
			fmx -= kx;
			fmy -= ky;
			fmz -= kz;
	}
	
	// Tilt compensation
	//float Xh = fmx * Mpu.cosPitch - fmz * Mpu.sinPitch;
	//float Yh = fmx * Mpu.sinRoll * Mpu.sinPitch + fmy * Mpu.cosRoll - fmz * Mpu.sinRoll * Mpu.cosPitch;
	
	//heading = (float)atan2(Yh, Xh);
	//log();
	log_sens();

}

#endif

void HmcClass::newCalibration(int16_t sh[]){

	const int if_no_change_cnt_to_stop = 2500;
	//wdt_enable(WDTO_4S);
	sh[0] = sh[1] = sh[2] = sh[3] = sh[4] = sh[5] = 0;
	printf("START ROTATION\n");

	delay(2000);
	int16_t mx, my, mz;
	long cnt = 0;
	long fcnt = 0;
	int print_time = millis_();
	while (true){
		fcnt++;
		bool new_val = false;
		readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
		if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
		mx = (((int16_t)buffer[0]) << 8) | buffer[1];
		my = (((int16_t)buffer[4]) << 8) | buffer[5];
		mz = (((int16_t)buffer[2]) << 8) | buffer[3];

		if (mx > sh[0]){
			sh[0] = mx;
			//printf("max_X: %i\n",mx);
			new_val = true;
		}
		if (mx < sh[1]){
			sh[1] = mx;
			//printf("min_X: %i\n", mx);
			new_val = true;
		}

		if (my > sh[2]){
			sh[2] = my;
			//printf("max_Y: %i\n", my);
			new_val = true;
		}
		if (my < sh[3]){
			sh[3] = my;
			//printf("min_Y: %i\n", my);
			new_val = true;
		}
		if (mz > sh[4]){
			sh[4] = mz;
			//printf("max_Z: %i\n", mz);
			new_val = true;
		}
		if (mz < sh[5]){
			sh[5] = mz;
			//printf("min_Z: %i\n", mz);
			new_val = true;
		}

		delay(10);
		if (new_val) {
			print_time = millis_()+1000;
			int i = 0;
			cout <<"X: "<< sh[i++] << "\t" << sh[i++] << endl;
			cout <<"Y: "<< sh[i++] << "\t" << sh[i++] << endl;
			cout <<"Z: "<< sh[i++] << "\t" << sh[i++] << endl;
			cout <<fcnt<<"\t------------"<< endl;
			cnt = 0;
		}
		else {
			if (millis_() > print_time) {
				cout << if_no_change_cnt_to_stop - cnt << endl;
				print_time += 1000;
			}
		}
		if (cnt++ > if_no_change_cnt_to_stop)
			break;
		//wdt_reset();
	}
	cout << "Stop Rottation\n";
	
}

bool HmcClass::calibration(const bool newc){

	if (newc && Autopilot.motors_is_on())
		return false;

	int16_t sh[6];

	if (newc){
		newCalibration(sh);
		Settings.saveCompssSettings(sh);
		//wdt_enable(WDTO_120MS);//reset
		cout << "RESET HMC\n";
	}

	calibrated = Settings.readCompassSettings(sh);
	if (calibrated == false) 
		cout<< "! ! ! ! Comppas not Calibrated ! ! ! !\n";
	bool mot_cal = Settings.readCompasMotorSettings(base);
	if (!mot_cal)
		cout << "! ! ! ! Comppas motors not Calibrated ! ! ! !\n";
	calibrated &= mot_cal;

	
	dx = (float)(sh[0] - sh[1])*0.5f;
	c_base[X] = (int16_t)(dx + sh[1]);
	cout << sh[0] << "\t" << sh[1] << endl;
	dx = 1.0f / dx;

	dy = (float)(sh[2] - sh[3])*0.5f;
	c_base[Y] = (int16_t)(dy + sh[3]);
	cout << sh[2] << "\t" << sh[3] << endl;
	dy = 1.0f / dy;

	dz = (float)(sh[4] - sh[5])*0.5f;
	c_base[Z] = (int16_t)(dz + sh[5]);
	cout << sh[4] << "\t" << sh[5] << endl;
	dz = 1.0f / dz;

	log_base();

	return true;
}



/** Default constructor, uses default I2C address.
* @see HMC5883L_DEFAULT_ADDRESS
*/
HmcClass::HmcClass() {
	devAddr = HMC5883L_DEFAULT_ADDRESS;
}

/** Specific address constructor.
* @param address I2C address
* @see HMC5883L_DEFAULT_ADDRESS
* @see HMC5883L_ADDRESS
*/
HmcClass::HmcClass(uint8_t address) {
	devAddr = address;
}

/** Power on and prepare for general usage.
* This will prepare the magnetometer with default settings, ready for single-
* use mode (very low power requirements). Default settings include 8-sample
* averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
* terms of LSB/Gauss). Be sure to adjust any settings you need specifically
* after initialization, especially the gain settings if you happen to be seeing
* a lot of -4096 values (see the datasheet for mor information).
*/
void HmcClass::initialize() {
	// write CONFIG_A register
	writeByte(devAddr, HMC5883L_RA_CONFIG_A,
		(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1)) |
		(HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)) |
		(HMC5883L_BIAS_NORMAL << (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

	// write CONFIG_B register
	setGain(HMC5883L_GAIN_1090);

	// write MODE register
	setMode(HMC5883L_MODE_SINGLE);
}

/** Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return True if connection is valid, false otherwise
*/
bool HmcClass::testConnection() {
	if (readBytes(devAddr, HMC5883L_RA_ID_A, 3, buffer) == 3) {
		return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
	}
	return false;
}

// CONFIG_A register

/** Get number of samples averaged per measurement.
* @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
* @see HMC5883L_AVERAGING_8
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_AVERAGE_BIT
* @see HMC5883L_CRA_AVERAGE_LENGTH
*/
uint8_t HmcClass::getSampleAveraging() {
	readBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, buffer);
	return buffer[0];
}
/** Set number of samples averaged per measurement.
* @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_AVERAGE_BIT
* @see HMC5883L_CRA_AVERAGE_LENGTH
*/
void HmcClass::setSampleAveraging(uint8_t averaging) {
	writeBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT, HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** Get data output rate value.
* The Table below shows all selectable output rates in continuous measurement
* mode. All three channels shall be measured within a given output rate. Other
* output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
* interrupt pin in single measurement mode.
*
* Value | Typical Data Output Rate (Hz)
* ------+------------------------------
* 0     | 0.75
* 1     | 1.5
* 2     | 3
* 3     | 7.5
* 4     | 15 (Default)
* 5     | 30
* 6     | 75
* 7     | Not used
*
* @return Current rate of data output to registers
* @see HMC5883L_RATE_15
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_RATE_BIT
* @see HMC5883L_CRA_RATE_LENGTH
*/
uint8_t HmcClass::getDataRate() {
	readBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, buffer);
	return buffer[0];
}
/** Set data output rate value.
* @param rate Rate of data output to registers
* @see getDataRate()
* @see HMC5883L_RATE_15
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_RATE_BIT
* @see HMC5883L_CRA_RATE_LENGTH
*/
void HmcClass::setDataRate(uint8_t rate) {
	writeBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT, HMC5883L_CRA_RATE_LENGTH, rate);
}
/** Get measurement bias value.
* @return Current bias value (0-2 for normal/positive/negative respectively)
* @see HMC5883L_BIAS_NORMAL
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_BIAS_BIT
* @see HMC5883L_CRA_BIAS_LENGTH
*/
uint8_t HmcClass::getMeasurementBias() {
	readBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, buffer);
	return buffer[0];
}
/** Set measurement bias value.
* @param bias New bias value (0-2 for normal/positive/negative respectively)
* @see HMC5883L_BIAS_NORMAL
* @see HMC5883L_RA_CONFIG_A
* @see HMC5883L_CRA_BIAS_BIT
* @see HMC5883L_CRA_BIAS_LENGTH
*/
void HmcClass::setMeasurementBias(uint8_t bias) {
	writeBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT, HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** Get magnetic field gain value.
* The table below shows nominal gain settings. Use the "Gain" column to convert
* counts to Gauss. Choose a lower gain value (higher GN#) when total field
* strength causes overflow in one of the data output registers (saturation).
* The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
*
* Value | Field Range | Gain (LSB/Gauss)
* ------+-------------+-----------------
* 0     | +/- 0.88 Ga | 1370
* 1     | +/- 1.3 Ga  | 1090 (Default)
* 2     | +/- 1.9 Ga  | 820
* 3     | +/- 2.5 Ga  | 660
* 4     | +/- 4.0 Ga  | 440
* 5     | +/- 4.7 Ga  | 390
* 6     | +/- 5.6 Ga  | 330
* 7     | +/- 8.1 Ga  | 230
*
* @return Current magnetic field gain value
* @see HMC5883L_GAIN_1090
* @see HMC5883L_RA_CONFIG_B
* @see HMC5883L_CRB_GAIN_BIT
* @see HMC5883L_CRB_GAIN_LENGTH
*/
uint8_t HmcClass::getGain() {
	readBits(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT, HMC5883L_CRB_GAIN_LENGTH, buffer);
	return buffer[0];
}
/** Set magnetic field gain value.
* @param gain New magnetic field gain value
* @see getGain()
* @see HMC5883L_RA_CONFIG_B
* @see HMC5883L_CRB_GAIN_BIT
* @see HMC5883L_CRB_GAIN_LENGTH
*/
void HmcClass::setGain(uint8_t gain) {
	// use this method to guarantee that bits 4-0 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	writeByte(devAddr, (uint8_t)HMC5883L_RA_CONFIG_B, (uint8_t)(gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1)));
}

// MODE register

/** Get measurement mode.
* In continuous-measurement mode, the device continuously performs measurements
* and places the result in the data register. RDY goes high when new data is
* placed in all three registers. After a power-on or a write to the mode or
* configuration register, the first measurement set is available from all three
* data output registers after a period of 2/fDO and subsequent measurements are
* available at a frequency of fDO, where fDO is the frequency of data output.
*
* When single-measurement mode (default) is selected, device performs a single
* measurement, sets RDY high and returned to idle mode. Mode register returns
* to idle mode bit values. The measurement remains in the data output register
* and RDY remains high until the data output register is read or another
* measurement is performed.
*
* @return Current measurement mode
* @see HMC5883L_MODE_CONTINUOUS
* @see HMC5883L_MODE_SINGLE
* @see HMC5883L_MODE_IDLE
* @see HMC5883L_RA_MODE
* @see HMC5883L_MODEREG_BIT
* @see HMC5883L_MODEREG_LENGTH
*/
uint8_t HmcClass::getMode() {
	readBits(devAddr, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT, HMC5883L_MODEREG_LENGTH, buffer);
	return buffer[0];
}
/** Set measurement mode.
* @param newMode New measurement mode
* @see getMode()
* @see HMC5883L_MODE_CONTINUOUS
* @see HMC5883L_MODE_SINGLE
* @see HMC5883L_MODE_IDLE
* @see HMC5883L_RA_MODE
* @see HMC5883L_MODEREG_BIT
* @see HMC5883L_MODEREG_LENGTH
*/
void HmcClass::setMode(uint8_t newMode) {
	// use this method to guarantee that bits 7-2 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	writeByte(devAddr, HMC5883L_RA_MODE, newMode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
* In the event the ADC reading overflows or underflows for the given channel,
* or if there is a math overflow during the bias measurement, this data
* register will contain the value -4096. This register value will clear when
* after the next valid measurement is made. Note that this method automatically
* clears the appropriate bit in the MODE register if Single mode is active.
* @param x 16-bit signed integer container for X-axis heading
* @param y 16-bit signed integer container for Y-axis heading
* @param z 16-bit signed integer container for Z-axis heading
* @see HMC5883L_RA_DATAX_H
*/
void HmcClass::getHeading(int16_t *x, int16_t *y, int16_t *z) {
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = (int16_t)((((int16_t)buffer[0]) << 8) | buffer[1]);
	*y = (int16_t)((((int16_t)buffer[4]) << 8) | buffer[5]);
	*z = (int16_t)((((int16_t)buffer[2]) << 8) | buffer[3]);
}
/** Get X-axis heading measurement.
* @return 16-bit signed integer with X-axis heading
* @see HMC5883L_RA_DATAX_H
*/
int16_t HmcClass::getHeadingX() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (int16_t)((((int16_t)buffer[0]) << 8) | buffer[1]);
}
/** Get Y-axis heading measurement.
* @return 16-bit signed integer with Y-axis heading
* @see HMC5883L_RA_DATAY_H
*/
int16_t HmcClass::getHeadingY() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (int16_t)((((int16_t)buffer[4]) << 8) | buffer[5]);
}
/** Get Z-axis heading measurement.
* @return 16-bit signed integer with Z-axis heading
* @see HMC5883L_RA_DATAZ_H
*/
int16_t HmcClass::getHeadingZ() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	readBytes(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE) writeByte(devAddr, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (int16_t)((((int16_t)buffer[2]) << 8) | buffer[3]);
}

// STATUS register

/** Get data output register lock status.
* This bit is set when this some but not all for of the six data output
* registers have been read. When this bit is set, the six data output registers
* are locked and any new data will not be placed in these register until one of
* three conditions are met: one, all six bytes have been read or the mode
* changed, two, the mode is changed, or three, the measurement configuration is
* changed.
* @return Data output register lock status
* @see HMC5883L_RA_STATUS
* @see HMC5883L_STATUS_LOCK_BIT
*/
bool HmcClass::getLockStatus() {
	readBit(devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, buffer);
	return buffer[0];
}
/** Get data ready status.
* This bit is set when data is written to all six data registers, and cleared
* when the device initiates a write to the data output registers and after one
* or more of the data output registers are written to. When RDY bit is clear it
* shall remain cleared for 250 us. DRDY pin can be used as an alternative to
* the status register for monitoring the device for measurement data.
* @return Data ready status
* @see HMC5883L_RA_STATUS
* @see HMC5883L_STATUS_READY_BIT
*/
bool HmcClass::getReadyStatus() {
	readBit(devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, buffer);
	return buffer[0];
}

// ID_* registers

/** Get identification byte A
* @return ID_A byte (should be 01001000, ASCII value 'H')
*/
uint8_t HmcClass::getIDA() {
	readByte(devAddr, HMC5883L_RA_ID_A, buffer);
	return buffer[0];
}
/** Get identification byte B
* @return ID_A byte (should be 00110100, ASCII value '4')
*/
uint8_t HmcClass::getIDB() {
	readByte(devAddr, HMC5883L_RA_ID_B, buffer);
	return buffer[0];
}
/** Get identification byte C
* @return ID_A byte (should be 00110011, ASCII value '3')
*/
uint8_t HmcClass::getIDC() {
	readByte(devAddr, HMC5883L_RA_ID_C, buffer);
	return buffer[0];
}


string HmcClass::get_calibr_set() {
	ostringstream convert;
	convert << \
		c_base[0] << "," << \
		c_base[1] << "," << \
		c_base[2] << "," << \
		dx << "," << \
		dy << "," << \
		dz;
	for (int i = 0; i < 12; i++) {
		convert << "," << base[i];
	}
	
	string ret = convert.str();
	return string(ret);









}


HmcClass Hmc;

