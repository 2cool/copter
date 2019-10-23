/*
message for LOG

MXT     max throttle
OFT		off_throttle
MD1		motors_do_on
MD0		motors do off
LWV		low voltage
GPE		gps error
CNF		controled fall
MXG		перегрузка
MODXXX  резим работі автопилота,XXX - число - control_bits

TFR	out of Perimetr gps
THG out of Perimetr high



*/

//при shift yaw на 180 грудусов летает в обратном направление при управлении наклоном телефона
//при старте его кидает в сторону. наверное проблема в необнулении стаб спид





//    Addition options -DDEBUG
#ifdef DEBUG
#define CALIBRATION__TIMEOUT 6e3
#else
#define CALIBRATION__TIMEOUT 60e3
#endif

#include "define.h"
#include "WProgram.h"

#include "mi2c.h"
#include "MS5611.h"
#include "Autopilot.h"
#include "Balance.h"

#include "commander.h"

#include "GPS.h"

#include "Settings.h"
#include "Telemetry.h"
#include "Stabilization.h"
#include "debug.h"
#include "Prog.h"

#include "mi2c.h"

#include <cstdio>
#include <signal.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <stdio.h>




#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include  "Log.h"


using namespace std;


//#define DEFAULT_STATE (Z_STAB|XY_STAB)
//#define DEFAULT_STATE 0 

bool set_alt = true;

void AutopilotClass::init(){/////////////////////////////////////////////////////////////////////////////////////////////////

	hall_ok = 255;
	shmPTR->sim800_reset = false;
	time_at__start = old_time_at__start = 0;
	lowest_height = shmPTR->lowest_altitude_to_fly;
	last_time_data__recived = 0;
	min_hor_accuracy_2_start = MIN_ACUR_HOR_POS_2_START_;
	ignore_the_lack_of_internet_at_startup = false;
	fall_thr = FALLING_THROTTLE;
	sens_z = 6;
	sens_xy = 0.285f;

	newData = false;

	//holdLocationStartStop(false);
	//holdAltitudeStartStop(false);

	height_to_lift_to_fly_to_home = HIGHT_TO_LIFT_ON_TO_FLY_TO_HOME;
	aPitch = aRoll = aYaw_=0;

	//was_connected_to_wifi = NO_WIFI_WATCH_DOG_IN_SECONS < 30;
	control_bits = Z_STAB | XY_STAB;
	old_control_bits = 0;
	aPitch = aRoll = 0;
	control_DeltaTime = 0;

	gimbalRoll= gimbalPitch=0;
	mega_i2c.gimagl(gimbalPitch, gimbalRoll);

}
int32_t AutopilotClass::powerOnTime() {
	return (time_at__start)?(millis_() - time_at__start):0;
}
float AutopilotClass::corectedAltitude4tel() {
	return Mpu.get_Est_Alt();
	//return ((control_bits & Z_STAB) == 0) ? Mpu.get_Est_Alt() : Stabilization.getAltitude();
}


void AutopilotClass::add_2_need_yaw(float speed, const float dt){ 
	aYaw_ += speed*dt;
	aYaw_ = wrap_180(aYaw_);
}


void AutopilotClass::hall_test() {
	if (hall_ok == 255)
		hall_ok = 0;
	for (int i = 0; i < 4; i++)
		hall_ok |= ((Telemetry.get_current(i) > 0.55))<<i;
}


void AutopilotClass::add_2_need_altitude(float speed, const float dt){
	
	float speedP=0, speedM=0;
	if (speed != 0) {
		set_alt = true;
	//	if (speed < -0.2 && (flyAtAltitude_R < lowest_height || flyAtAltitude_V < lowest_height))
		//	speed = fmax(-0.2, speed);

		if (speed > 0) 
			speedP = speed;
		else 
			speedM = speed;
		Stabilization.set_max_sped_ver(speedP, speedM);

		flyAtAltitude_V = Stabilization.getDist_Z(speedP+speedM);
		flyAtAltitude_R += speed * dt;
		flyAtAltitude_V += flyAtAltitude_R;
	}
	else {
		if (set_alt) {
			set_alt = false;
			Stabilization.set_max_sped_ver(speedP, speedM);
			flyAtAltitude_V = flyAtAltitude_R= Mpu.get_Est_Alt();
			
		}
	}
}
//-------------------------------------------------------------------------
void AutopilotClass::smart_commander(const float dt){
	////if (Commander.getPitch() != 0 || Commander.getRoll() != 0){
		const float addX = sens_xy*(Commander.getPitch());
		const float addY = -sens_xy*(Commander.getRoll());
		const float cyaw = Commander.getYaw()*GRAD2RAD;
		const float cosL = (float)cos(cyaw);
		const float sinL = (float)sin(cyaw);
		float speedX = addX * cosL + addY *sinL;
		float speedY = -(addX * sinL - addY *cosL);

		Stabilization.add2NeedPos(speedX, speedY, dt);

	

		
	//}
	//else{
	//	GPS.loc.setSpeedZero();
	//}
}

static int32_t last_beep__time = 0;



void AutopilotClass::loop(){/////////////////////////////////////////////////////////////////////////////////////////////////

	
	
	const int32_t _ct = millis_();
	float dt = (float)((_ct - control_DeltaTime)*1e-3); 
	if (dt < 0.033)
		return;
	dt= constrain(dt, 0.033, 0.1);
	control_DeltaTime = _ct;

	if (shmPTR->control_bits_4_do)
		set_control_bits(shmPTR->control_bits_4_do);
	shmPTR->control_bits_4_do = 0;
	shmPTR->control_bits = control_bits;


	
	gimBalRollCorrection();

	


#ifdef LOST_BEEP
	
	if (last_time_data__recived &&  GPS.loc._lat_zero!=0 && GPS.loc._lon_zero!=0  && (_ct - last_time_data__recived)>3e3 && (_ct - last_beep__time) > 3e3) {
		last_beep__time = _ct;
		if ((_ct - last_time_data__recived) < 5e3)
			Telemetry.addMessage(e_LOST_CONNECTION);
		mega_i2c.beep_code(B_CONNECTION_LOST);
	}
#endif

	//-----------------------------------------
	static int uspd_cnt = 0;
	if (motors_is_on() && fabs(Mpu.get_roll()) > 110 && Mpu.get_Est_Alt() < 5) {
		if (uspd_cnt++ > 10)
			off_throttle(true, "USD");
	}else
		uspd_cnt = 0;
	//-----------------------------------------

	if (MS5611.fault() && motors_is_on() && go2homeState() == 0) {
		//sim.send_sos(e_BARROMETR_FAULT);
		going2HomeON(true);
	}



	if (control_bits&CONTROL_FALLING){
		aYaw_ = Mpu.get_yaw();
		off_throttle(false,"cntr_fall");
	}
	else{
		if (control_bits & PROGRAM){
			
			Prog.loop();
		}
		else{
			if (control_bits & GO2HOME){
				go2HomeProc(dt);
			}
			else{
				int32_t timelag = _ct - last_time_data__recived;
				if (last_time_data__recived && motors_is_on() && timelag > CONNECTION_LOST__TIMEOUT) {
					connectionLost_();
					return;
				}
#ifndef OFF_TIMELAG
				if (timelag > TIMEOUT__LAG) {
					int32_t _timeout__LAG = TIMEOUT__LAG;
					if (Mpu.get_Est_Alt() > 15 || Mpu.get_Est_Alt() / (-Mpu.get_Est_SpeedZ()) > 5)
						_timeout__LAG = 2e3;
					if (timelag > _timeout__LAG)
						Commander.data_reset();
				}
#endif
				
				aYaw_ = Commander.get_yaw_minus_offset();
				if (control_bits & Z_STAB){
					const float thr = Commander.getThrottle();
					const float speed = (thr - MIDDLE_POSITION) * sens_z;
					add_2_need_altitude(speed, dt);
				}
				else{
					throttle = Commander.getThrottle();
				}
				if (control_bits & XY_STAB){
					smart_commander(dt);
				}
				else{
					aPitch = Commander.getPitch();
					aRoll = Commander.getRoll();
				}
			}
		}
			
	}

	if (_ct < CALIBRATION__TIMEOUT || Mpu.acc_callibr_time > micros_()) {
		mega_i2c.set_led_mode(2, 5, true);
	}
	else {

		if (motors_is_on() == false) {
			if (is_all_OK(false))
				mega_i2c.set_led_mode(0, 10, false);
			else
				mega_i2c.set_led_mode(0, 10, true);
		}
		else {
			if (control_bits&CONTROL_FALLING)
				mega_i2c.set_led_mode(2, 255, false);
			else {
				int n = ((control_bits&(XY_STAB | Z_STAB)) == (XY_STAB | Z_STAB)) ? 3 : 1;
				mega_i2c.set_led_mode(n, 255, (control_bits & GO2HOME) | (control_bits & PROGRAM));
			}

		}
	}



	if (shmPTR->sim800_reset) {
		shmPTR->sim800_reset = false;
		mega_i2c.sim800_reset();
	}



	log();

}

void AutopilotClass::log() {
	if (old_control_bits != control_bits && Log.writeTelemetry) {
		Log.block_start(LOG::AUTO);
		Log.loaduint32t(control_bits);
		old_control_bits = control_bits;
		Log.block_end();
	}
}
string AutopilotClass::get_set(bool for_save){
	
	ostringstream convert;
	convert << \
		height_to_lift_to_fly_to_home << "," << \
		Balance.get_max_throttle() << "," << \
		Balance.get_min_throttle() << "," << \
		sens_xy << "," << \
		sens_z << "," << \
		lowest_height << "," << \
		shmPTR->fly_at_start << "," << \
		Telemetry.get_bat_capacity() << ",";
	if (for_save)
		convert << MIN_ACUR_HOR_POS_2_START_ << "," << 0;
	else
		convert << min_hor_accuracy_2_start << "," << ignore_the_lack_of_internet_at_startup;

	string ret = convert.str();
	return string(ret);
}

void AutopilotClass::set(const float ar[]){
	cout << "Autopilot set\n";
	int i = 0;
	if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK){
		Settings.set(ar[i++], height_to_lift_to_fly_to_home);
		Balance.set_min_max_throttle(ar[i++], ar[i++]);
		//i += 2;
		Settings.set(ar[i++], sens_xy);
		Settings.set(ar[i++], sens_z);
		Settings.set(ar[i++], lowest_height);
		Settings.set(ar[i++], shmPTR->fly_at_start);
		if (shmPTR->fly_at_start + 1 < lowest_height)
			shmPTR->fly_at_start = lowest_height + 1;
		if (motors_is_on()==false)
			Telemetry.set_bat_capacity(ar[i++]);
		min_hor_accuracy_2_start = ((ar[i] > MIN_ACUR_HOR_POS_4_JAMM) ? MIN_ACUR_HOR_POS_4_JAMM : ar[i]);
		i++;
		ignore_the_lack_of_internet_at_startup = (ar[i++] > 0);

		int ii;
		for (ii = 0; ii < i; ii++){
			cout << ar[ii] << ",";
		}
		cout << ar[ii] << endl;
	}
	
}







void AutopilotClass::set_new_altitude(float alt){
	set_alt = true;
	flyAtAltitude_R = flyAtAltitude_V = alt;
}

bool AutopilotClass::holdAltitude(float alt){
	flyAtAltitude_R = flyAtAltitude_V = alt;
	control_bits |= Z_STAB;
	//setbuf(stdout, NULL);
	cout << "FlyAt: " << flyAtAltitude_V << "\t"<<millis_() << endl;

	return true;
}

bool AutopilotClass::holdAltitudeStartStop(){
	set_alt = true;
	if (!motors_onState() || go2homeState() || progState())
		return false;
	bool h = (control_bits & Z_STAB)==0;
	if (h){
		Stabilization.resset_z();
		return holdAltitude(Mpu.get_Est_Alt());
	}
	else{
		control_bits ^= Z_STAB;
		throttle = HOVER_THROTHLE;
		Stabilization.resset_z();
		return true;
		
	}
	return false;
}
//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
enum{ JUMP = 0, HOWER = 1, GO_UP_OR_NOT = 2, TEST_ALT1 = 3, GO2HOME_LOC = 4, TEST4HOME_LOC = 5, START_FAST_DESENDING = 6, TEST_ALT2 = 7,SLOW_DESENDING=8 };
bool AutopilotClass::go2HomeProc(const float dt){
	 switch (go2homeIndex){
		case JUMP:{	
			dist2home_at_begin = GPS.loc.dist2home;
			if (Mpu.get_Est_Alt() < 3)
				holdAltitude(3);
			go2homeIndex=HOWER;
			break;
		}
		case HOWER:{	//висеть 20 секунд
			f_go2homeTimer += dt;
			if (f_go2homeTimer > ((howeAt2HOME)?HOWER_TIME:4)){ //for stabilization and connection
				go2homeIndex = GO_UP_OR_NOT;
			}
			break;
		}
		case GO_UP_OR_NOT:{
			const float accuracy = ACCURACY_XY + GPS.loc.accuracy_hor_pos_;
			if (fabs(Mpu.get_Est_X()) <= accuracy && fabs(Mpu.get_Est_Y()) <= accuracy){ // if at home now
				f_go2homeTimer = 6; //min time for stab
				go2homeIndex = (Mpu.get_Est_Alt() <= (FAST_DESENDING_TO_HIGH)) ? SLOW_DESENDING : START_FAST_DESENDING;
				Stabilization.setNeedPos2Home();
				break;
			}	
			//поднятся  на высоту  X если ниже, опустится если више 
			if (Mpu.get_Est_Alt() > DOWN_IF_HIGHER_THEN_ON_FLY_TO_HOME)
				flyAtAltitude_R = flyAtAltitude_V = DOWN_IF_HIGHER_THEN_ON_FLY_TO_HOME;
			else if (Mpu.get_Est_Alt()< height_to_lift_to_fly_to_home)
				flyAtAltitude_R=flyAtAltitude_V = height_to_lift_to_fly_to_home;
			go2homeIndex = TEST_ALT1;
			break;
		}
		case TEST_ALT1:{
			if (fabs(Mpu.get_Est_Alt() - flyAtAltitude_V) <= (ACCURACY_Z)){
				go2homeIndex = GO2HOME_LOC;
			}
			break;
		}
		case GO2HOME_LOC:{//перелететь на место старта
			// led_prog = 4;
			Stabilization.setNeedPos2Home();
			go2homeIndex = TEST4HOME_LOC;
			aYaw_ = -RAD2GRAD * atan2(Mpu.get_Est_Y(),Mpu.get_Est_X());// GPS.loc.dir_angle_GRAD;

			aYaw_ = wrap_180(aYaw_);
			break;
		}
		case TEST4HOME_LOC:{//прилет на место старта
			   
			const float accuracy = ACCURACY_XY + GPS.loc.accuracy_hor_pos_;
			if (fabs(Mpu.get_Est_X()) <= accuracy && fabs(Mpu.get_Est_Y()) <= accuracy){
				go2homeIndex = START_FAST_DESENDING;
				f_go2homeTimer = 0;
			}
			break;
		}
		case START_FAST_DESENDING:
			f_go2homeTimer += dt;
			if (f_go2homeTimer > 5){
				go2homeIndex = TEST_ALT2;
				flyAtAltitude_R = flyAtAltitude_V = (FAST_DESENDING_TO_HIGH);
			}
			break;


		case TEST_ALT2:{//спуск до FAST_DESENDING_TO_HIGH метров
			if (fabs(Mpu.get_Est_Alt() - flyAtAltitude_V) < (ACCURACY_Z)){
				go2homeIndex = SLOW_DESENDING;
			}
			   
			break;
		}
		case SLOW_DESENDING:
		{ 
			if (MS5611.fault()) {
				control_falling(e_BARROMETR_FAULT);
				return false;
			}
			//плавній спуск
			if (Mpu.get_Est_Alt() >lowest_height){
				float k = Mpu.get_Est_Alt()*0.05f;
				if (k < 0.1f)
					k = 0.1f;
				flyAtAltitude_V -= (dt*k);
				flyAtAltitude_R = flyAtAltitude_V;
			}
			else{
				flyAtAltitude_R = flyAtAltitude_V = lowest_height;
			}
						   
			break;
		}
	 }

	if (GPS.loc.dist2home - dist2home_at_begin > MAX_DIST_ERROR_TO_FALL){
		cout << GPS.loc.dist2home - dist2home_at_begin<<" STRONG_WIND\n";
		Autopilot.off_throttle(false, e_TOO_STRONG_WIND);
	}

	return true;
}
bool AutopilotClass::going2HomeON(const bool hower){
	
	Stabilization.setDefaultMaxSpeeds();

	howeAt2HOME = hower;//зависнуть на месте или нет

	bool res = holdAltitude(Mpu.get_Est_Alt());
	res &= holdLocation(GPS.loc.lat_, GPS.loc.lon_);
	if (res){
		control_bits |= GO2HOME;
		f_go2homeTimer = 0;
		//Out.println("Hanging on the site!");
		cout << "go2home" << "\t"<< millis_() << endl;
		go2homeIndex=JUMP;
	}
	return res;
}

bool AutopilotClass::going2HomeStartStop(const bool hower){
	set_alt = true;
	if (!motors_onState())
		return false;
	bool f = (control_bits & GO2HOME);
	if (f){
		flyAtAltitude_R = flyAtAltitude_V;
		control_bits ^=GO2HOME;
		holdLocation(GPS.loc.lat_, GPS.loc.lon_);
		return true;
	}
	else{
		if (progState())
			start_stop_program(true);
		return going2HomeON(hower);
	}
}


bool AutopilotClass::holdLocation(const long lat, const long lon){
	
	aPitch = aRoll = 0;
	//if (holdAltitude()){

	
		//GPS.loc.setNeedLoc(lat,lon);
		cout << "Hower at: " << GPS.loc.lat_ << " " << GPS.loc.lon_ << "\t"<< millis_() << endl;;

		//Stabilization.init_XY(0, 0);
		Stabilization.setNeedPos(Mpu.get_Est_X(), Mpu.get_Est_Y());

		control_bits |= XY_STAB;
		return true;
	//}
	//else
	//	return false;
}

bool AutopilotClass::holdLocationStartStop(){/////////////////////////////////////////////////////////////////////////////////////////////////
	if (!motors_onState() || go2homeState() || progState())
		return false;
	bool h = (control_bits &XY_STAB)==0;
	if (h){
		//Stabilization.resset_xy_integrator();
		return holdLocation(GPS.loc.lat_, GPS.loc.lon_);
	}
	else{
		control_bits ^=  XY_STAB;
		Stabilization.resset_xy_integrator();
		//holdAltitude();
		return true;
	}
	return false;
}
//-------------------------------------------------------------------------------------------------------------------------
bool AutopilotClass::is_all_OK(bool print){
	const int32_t _ct = millis_();
	//printf( "\MS5611 err: %f\n",MS5611.getErrorsK());
#ifndef DEBUG
	if (hall_ok<0b1111) {
		cout << "\n!!!hall_error!!!" << CALIBRATION__TIMEOUT - (int)_ct << " sec." << "\t" << _ct << endl;
		mega_i2c.beep_code(B_MS611_ERROR);
		return false;
	}
#endif
	

#ifndef FLY_EMULATOR
	
	if (_ct < CALIBRATION__TIMEOUT) {
		if (print) {
			cout << "\n!!!calibrating!!! to end:" << CALIBRATION__TIMEOUT - (int)_ct << " sec." << "\t" << _ct << endl;
			mega_i2c.beep_code(B_MS611_ERROR);
		}
		return false;
	}
#endif

	if (shmPTR->inet_ok == false) {
		if (print) {
			Telemetry.addMessage(e_NO_INTERNET);
			cout << "inet dont work" << "\t" << _ct << endl;
		}
#ifndef DEBUG
		if (!ignore_the_lack_of_internet_at_startup)
			return false;
#endif
	}

	if (Hmc.do_compass_motors_calibr || (Mpu.gyro_calibratioan && Hmc.calibrated_)) {
		if (Telemetry.low_voltage) {
			if (print) {
				Telemetry.addMessage(e_LOW_VOLTAGE);
				cout << " LOW VOLTAGE" << "\t" << _ct << endl;
				mega_i2c.beep_code(B_LOW_VOLTAGE);
			}
			return false;
		}

		if (Hmc.do_compass_motors_calibr == false && GPS.loc.accuracy_hor_pos_ > min_hor_accuracy_2_start) {
			if (print) {
				cout << " GPS error" << "\t" << _ct << endl;
				mega_i2c.beep_code(B_GPS_ACCURACY_E);
				Telemetry.addMessage(e_GPS_ERROR);
			}
#ifndef DEBUG
			return false;
#endif 
		}
		if (print)
			cout << "OK" << "\t" << _ct << endl;
		return true;


	}
	else {
		if (Hmc.calibrated_ == false) {
			if (print) {
				cout << "compas, ";
				mega_i2c.beep_code(4);
			}
		}
		if (Mpu.gyro_calibratioan == false) {
			if (print) {
				cout << "gyro";
				mega_i2c.beep_code(5);
			}

		}
		if (print)
			cout << " calibr FALSE" << "\t" << _ct << endl;
	}
	return false;
}
/*
beep codes
{0, B00001000, B00001001, B00001010, B00001011, B00001100, B00001101, B00001110, B00001111, B00000001, B00000010, B00000011, B00000100, B00000101, B00000110, B00000111 };//4 beeps. 0 short 1 long beep
*/
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

bool AutopilotClass::motors_do_on(const bool start, const string msg){////////////////////////  M O T O R S  D O  ON  /////////////////////////////////////////////////////////////////////////
	const int32_t _ct = millis_();
	if (_ct - old_time_at__start < 3000)
		return false;
	cout << msg << "-";
	
	if (start){


		cout << "ON\n";

		if (is_all_OK(true) == false)
			return false;

		time_at__start = _ct;
		Telemetry.update_voltage();
		control_bits |= MOTORS_ON;
		GPS.loc.setHomeLoc();
		Mpu.set_XYZ_to_Zero();  // все берем из мпу. при  старте x y z = 0;
		//tflyAtAltitude = flyAtAltitude = 0;// Mpu.get_Est_Alt();

		//if (control_bits&Z_STAB) 
		//	holdAltitude(shmPTR->fly_at_start);
		//if (control_bits&XYSTAB)
		//	holdLocation(GPS.loc.lat_, GPS.loc.lon_);
		Stabilization.resset_z();
		Stabilization.resset_xy_integrator();
		aYaw_ = -Mpu.get_yaw();
		Log.run_counter++;
		
	}//------------------------------OFF----------------
	else {
		if (time_at__start)
			Telemetry.on_power_time += _ct - time_at__start;
		old_time_at__start = _ct;
		time_at__start = 0;

		cout << "off ";
		Telemetry.addMessage(i_OFF_MOTORS);
		off_throttle(true, msg);

		cout << "OK" << "\t"<< _ct <<endl;

		//if (camera_mode) {//----------------------------------
		//	thread t(stop_video);
		//	t.detach();

		//}
	}
	return true;
}

void AutopilotClass::control_falling(const string msg){
	if (motors_is_on() && (control_bits & CONTROL_FALLING) == 0){
		throttle = fall_thr*Balance.powerK();
		aPitch = aRoll = 0;
#ifdef DEBUG_MODE
		printf( "CNTROLL FALLING\n");
#endif
		Telemetry.addMessage(msg);
		Telemetry.addMessage(i_CONTROL_FALL);
		control_bits = CONTROL_FALLING | MOTORS_ON;
	}
}

bool AutopilotClass::off_throttle(const bool force, const string msg){/////////////////////////////////////////////////////////////////////////////////////////////////
	if ( force)
	{
		cout << "force motors_off " << msg << ", alt: " << (int)Mpu.get_Est_Alt() << ", time " << (int)millis_() << endl;
		Balance.set_off_th_();
		Telemetry.addMessage(msg);
		control_bits &= -1 ^ (MOTORS_ON | GO2HOME | CONTROL_FALLING | PROGRAM);
		return true;
	}
	else{
	//	if (control_bits_ & (255 ^ (COMPASS_ON | HORIZONT_ON)))
	//		return true;

		//if (Mpu.get_Est_Alt()  < 2){
		//	motors_do_on(false,msg);
		//}
		//else{
			control_falling(msg);
			
			//Out.println(throttle);
		//}
	}
	return false;

}

void AutopilotClass::connectionLost_(){ ///////////////// LOST
	const int32_t _ct = millis_();
#ifdef FLY_EMULATOR
	//if (true)
	//	return;
#endif

	shmPTR->connected = 0;
	shmPTR->commander_buf_len = 0;
	shmPTR->telemetry_buf_len = 0;

	cout << "connection lost" << "\t"<<_ct<<endl;
	Telemetry.addMessage(e_LOST_CONNECTION);
	Commander.controls2zero();

#ifdef OFF_MOTOR_IF_LOST_CONNECTION
if (motors_is_on())
off_throttle(true, "lost connection");
return;
#endif

	if (motors_is_on())
		if (go2homeState() == false && progState() == false) {
			aPitch = aRoll = 0;

			if (going2HomeON(true) == false && (_ct - last_time_data__recived) > (CONNECTION_LOST__TIMEOUT*3)) {
				off_throttle(false, e_NO_GPS_2_LONG);
			}
		}
	
}
void AutopilotClass::calibration() {/////////////////////////////////////////////////////////////////////////////////////////////////
	
	cout << "Set Calibr NOT USET.\n";
	/*
	if (fabs(cPitch + Commander.pitch) > 0.1 || fabs(cRoll + Commander.roll) > 0.1)
		return;

	cPitch += Commander.pitch;
	Commander.pitch = 0;
	cRoll += Commander.roll;
	Commander.roll = 0;
	*/
}

void AutopilotClass::set_gimBalPitch(const float angle) {
	if (mega_i2c.gimagl(angle, gimbalRoll))
		gimbalPitch = angle;
}

void AutopilotClass::gimBalRollADD(const float add) {
	if (!progState())
		if (mega_i2c.gimagl((gimbalPitch), gimbalRoll + add))
			gimbalRoll += add;

}
void AutopilotClass::gimBalPitchADD(const float add) {
	if (!progState())
		if (mega_i2c.gimagl(( gimbalPitch + add), gimbalRoll))
			gimbalPitch += add;

}
void AutopilotClass::program_is_loaded(bool set) {
	if (set) 
		control_bits |= PROGRAM_LOADED;
	else
		control_bits &= -1 ^ PROGRAM_LOADED;
}
bool AutopilotClass::start_stop_program(const bool stopHere){
	set_alt = true;
	if (motors_is_on()) {
		if (progState()) {
			control_bits ^= PROGRAM;
			Prog.clear();
			Stabilization.setDefaultMaxSpeeds();
			if (stopHere) {
				float alt = Mpu.get_Est_Alt();
				if (alt < 10)
					alt = 10;
				holdAltitude(alt);
				holdLocation(GPS.loc.lat_, GPS.loc.lon_);
			}
			return true;
		}
		else {
			if (Prog.start()) {
				if (go2homeState())
					going2HomeStartStop(false);
				bool res = holdAltitude(Mpu.get_Est_Alt());
				res &= holdLocation(GPS.loc.lat_, GPS.loc.lon_);
				if (res) {
					control_bits |= PROGRAM;
					cout << "prog started" << "\t"<< millis_() << endl;;
					return true;
				}
			}
			return false;
		}
	}
	return false;
	
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
bool AutopilotClass::set_control_bits(uint32_t bits) {
	if (bits==0)
		return true;
	//	uint8_t mask = control_bits_^bits;
	//printf("comm=%i\n", bits);

	//cout << bits << endl;
	mega_i2c.beep_code(B_COMMAND_RECEIVED);

	if (MOTORS_ON&bits)  {
		Hmc.do_compass_motors_calibr = false;
		bool on = motors_is_on() == false;
		on = motors_do_on(on, m_START_STOP);
		if (on == false) {
			cout << "motors on denied!"<< "\t"<<millis_()<<endl;
		}
	}

	
	if (bits & GIMBAL_PLUS) {
		if (bits & GIMBAL_AXIS)
			gimBalRollADD(0.3);
		else
			gimBalPitchADD(1);
	}

	if (bits & GIMBAL_MINUS) {
		if (bits & GIMBAL_AXIS)
			gimBalRollADD(-0.3);
		else
			gimBalPitchADD(-1);
	}
	if (control_bits & CONTROL_FALLING) {
		cout << "<< CONTROL_FALLING " << millis_() << endl;
		return false;
	}

	if (control_bits&MOTORS_ON) {
		if (bits & GO2HOME) {
			going2HomeStartStop(false);
			cout << "<< GO2HOME "<<(!Autopilot.go2homeState()?"OFF ":"ON ") << millis_() << endl;
		}

		if (bits & PROGRAM) {
			start_stop_program(true);
			cout << "<< PROGRAM " << (!Autopilot.progState() ? "OFF " : "ON ") << millis_() << endl;
		}

		if (bits & Z_STAB) {
			holdAltitudeStartStop();
			cout << "<< Hold Alt " << (!Autopilot.z_stabState() ? "OFF " : "ON ") << millis_() << endl;
		}

		if (bits & XY_STAB) {
			holdLocationStartStop();
			cout << "<< XY stab " << (!Autopilot.xy_stabState() ? "OFF " : "ON ") << millis_() << endl;
		}
	}
	else 
		control_bits ^= bits & ( XY_STAB | Z_STAB);
	
	//-----------------------------------------------
	if (bits & (MPU_ACC_CALIBR | MPU_GYRO_CALIBR)) {
		if (millis_() > CALIBRATION__TIMEOUT) {
			control_bits |= (MPU_ACC_CALIBR | MPU_GYRO_CALIBR);
			Mpu.new_calibration(!(bits&MPU_ACC_CALIBR));
			control_bits &= (0xffffffff ^ (MPU_ACC_CALIBR | MPU_GYRO_CALIBR));
		}
	}
	if (bits & COMPASS_MOTOR_CALIBR) {
		Hmc.start_motor_compas_calibr();
		if (Hmc.do_compass_motors_calibr)
			control_bits |= COMPASS_MOTOR_CALIBR;
	}
	if (bits & COMPASS_CALIBR) {
		control_bits |= COMPASS_CALIBR;
		Hmc.calibration(true);
		control_bits &= ~COMPASS_CALIBR;
	}
	if (bits & SHUTDOWN)
	{
		shutdown();
	}
	if (bits & REBOOT)
	{
		reboot();
	}

	return true;
}



int  AutopilotClass::reboot() {
	if (motors_is_on() == false) {
		cout << "REBOOT" << "\t"<<millis_() << endl;
		shmPTR->reboot = 1;
		shmPTR->run_main = false;
		return 0;
	}else
		return -1;
}
int  AutopilotClass::shutdown() {
	if (motors_is_on() == false) {
		cout << "SHUTD" << "\t"<< millis_() << endl;
		shmPTR->reboot = 2;
		shmPTR->run_main = false;
		return 0;
	}
	else
		return -1;
}
int  AutopilotClass::exit() {
	if (motors_is_on() == false) {
		cout << "EXIT" << "\t"<< millis_() << endl;
		shmPTR->reboot = 3;
		shmPTR->run_main = false;
		return 0;
	}
	else
		return -1;
}






#define MAX_GIMAGL_PITCH 20
#define MAX_GIMBAL_ROLL 20
void AutopilotClass::gimBalRollCorrection() {
	const float roll = Mpu.get_roll();
	static float roll_correction=0;
	float rc;
	if (fabs(roll) > MAX_GIMBAL_ROLL)
		rc = -2 * (roll - ((roll > 0) ? MAX_GIMBAL_ROLL : -MAX_GIMBAL_ROLL));
	else
		rc = 0;

	if (roll_correction!=rc) {
		mega_i2c.gimagl((gimbalPitch), gimbalRoll + roll_correction);
		roll_correction = rc;
	}
}


AutopilotClass Autopilot;

