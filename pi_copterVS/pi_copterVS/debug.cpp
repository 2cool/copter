// 
// 
// 
#include "Settings.h"
#include "debug.h"
#include "define.h"

	 int64_t d_old_t = 0;
	 int d_delay = 50;
	 double d_dt = (double)d_delay *0.001;
	 // int cnt = 0;
	 int32_t old_time = 0;
void DebugClass::graphic(const int n, const float x, const float y) {
	printf("%i,%f,%f\n", (unsigned int)n, x, y);

}
void DebugClass::graphic(const int n, const float x, const float y,const float z) {
	printf("%i,%f,%f,%f\n", (unsigned int)n, x, y, z);

}
	 void DebugClass::init() { i = 0; }
	 void DebugClass::dump(const long f1, long f2, long f3, long f4) {
		 printf("\n%d,%d,%d,%d\n", f1, f2, f3, f4);
	 }
	 void DebugClass::dump(const double f1, double f2, double f3, double f4) {
#ifndef LOG_READER
		 int32_t t = millis_();
		 if (t - old_time < d_delay)//20)
			 return;
		 old_time = t;
#endif
		 printf("\n%f,%f,%f,%f\n", f1, f2, f3, f4);


	 }
	 void DebugClass::dump(const uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4) {



		 printf("%i,%i,%i,%i\n", (unsigned int)f1, (unsigned int)f2, (unsigned int)f3, (unsigned int)f4);

	 }


	 string DebugClass::get_set(bool for_save) {
		 ostringstream convert;

#ifdef FLY_EMULATOR
		 convert << \
			 Emu.get_windX() << "," << \
			 Emu.get_windY() << "," << \
			 Emu.get_windZ() << "," << \
			 4 << "," << \
			 5 << "," << \
			 6 << "," << \
			 7 << "," << \
			 8 << "," << \
			 9 << "," << \
			 10 ;
#else
		 convert << \
			 1 << "," << \
			 2 << "," << \
			 3 << "," << \
			 4 << "," << \
			 5 << "," << \
			 6 << "," << \
			 7 << "," << \
			 8 << "," << \
			 9 << "," << \
			 10;
#endif
			
		 string ret = convert.str();
		 return string(ret);
	 }
	 
	 void DebugClass::set(const float ar[]) {
		 int i = 0, j = 0;

		 

		 if (ar[SETTINGS_ARRAY_SIZE] == SETTINGS_IS_OK) {
#ifdef FLY_EMU
float wind[3] = { Emu.get_windX(),Emu.get_windY(),Emu.get_windZ() };
			 wind[j++] = ar[i++];
			 wind[j++] = ar[i++];
			 wind[j++] = ar[i++];
			 Emu.init_wind(wind[0], wind[1], wind[2]);
#else
		 Settings.set(ar[i++], sett[j++]);
		 Settings.set(ar[i++], sett[j++]);
		 Settings.set(ar[i++], sett[j++]);
#endif
			 Settings.set(ar[i++], sett[j++]);
			 Settings.set(ar[i++], sett[j++]);
			 Settings.set(ar[i++], sett[j++]);
			 Settings.set(ar[i++], sett[j++]);
			 Settings.set(ar[i++], sett[j++]);
			 Settings.set(ar[i++], sett[j++]);
			 Settings.set(ar[i++], sett[j++]);

		 }
	 }


	 void DebugClass::load(const uint8_t i, const float x, const float y) {
		 if (d_old_t == 0) {
			 d_old_t = micros_();
			 return;
		 }
		 int64_t t = micros_();
		 double dt = (double)(t - d_old_t);
		 d_old_t = t;
		 dt *= 0.000001;


		 double F = dt / d_dt;
		 F = constrain(F,0.1, 1);
		 F = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 ar[i][0] += (x-ar[i][0])*F;
		 ar[i][1] += (y-ar[i][1])*F;
	 }


	 void DebugClass::load(const uint8_t i, const float x, const float y,const float z) {
		 if (d_old_t == 0) {
			 d_old_t = micros_();
			 return;
		 }
		 int64_t t = micros_();
		 double dt = (double)(t - d_old_t);
		 d_old_t = t;
		 dt *= 0.000001;


		 double F = dt / d_dt;
		 F = constrain(F, 0.1, 1);
		 F = 1;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 ar[i][0] += (x - ar[i][0])*F;
		 ar[i][1] += (y - ar[i][1])*F;
		 ar[i][2] += (z - ar[i][2])*F;


	 }

	 void DebugClass::dump(bool thre) {//--------------------------------------------------------------
		 if (n_debug > 9)
			 return;
		 int32_t t = millis_();
		 if (t - old_time < d_delay)//20)
			 return;
		 old_time = t;

		 /*
		 int n = 0;//Out.available();
		 if (n > 3) {
			 float f = readSeralFloat();
			 Out.println(f);
			 Mpu.temp_deb = f;
			 // Mpu.faccX.setF(f);
			 //	 Mpu.faccY.setF(f);
			 //	 Mpu.faccZ.setF(f);
		 }
		 else {
			 int b = 0;//Out.read();
			 if (b >= '0' && b <= '9')
				 i = b - '0';
		 }
		 //if (((++cnt) & 7) == 0){
		 */
		 i = n_debug;
		 if (thre == false)
			 graphic(i, ar[i][0], ar[i][1]);
		 else
			 graphic(i, ar[i][0], ar[i][1], ar[i][2]);
		 // }
		 // Out.println(Mpu.temp_deb);
	 }
DebugClass Debug;

