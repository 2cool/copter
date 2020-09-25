#ifndef WPROGRAM_
#define WPROGRAM_

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <thread>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iomanip>
#include <locale>
#include <sstream>
#include <fstream>


using namespace std;



#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))




//#define string std::string
#define bool bool

#define byte uint8_t
#define WDTO_2S 2
#define WDTO_1S 1
//void wdt_enable(int a){};
//void digitalWrite(int i1, int i2){}
//void pinMode(int i1, int i2){}
#define INPUT 0
#define OUTPUT 0
#define Serial2 Serial
#define Serial1 Serial
int32_t millis_();
int64_t micros_();

void delay(unsigned long);
std::string exec(const std::string s);


/*
const char * dtostre(float f, string ch[], int n, int fl=0){
		
		//int Number = 123;
		//string Result;
		ostringstream convert;
		convert << f;
		return convert.str();//.c_str();
		
	//cout<<Result<<endl;
	}
*/

/*
class Serial_class{
	public:
	void setTimeout(const int i){}
	void begin(int i){}
	void write(char* i, int ii){}
	bool find(char *c){return false;}
	bool available(){return false;}
	byte read(){return 0;}
	unsigned int printf(char s[]);
	unsigned int printf(char c);
	unsigned int printf(string &s);
	unsigned int printf(string s);
	unsigned int println(char s[]);
	unsigned int println(char c);
	//unsigned int println(string &s);
	unsigned int println(string s);
	unsigned int println(void);
	
};
*/





inline void sin_cos(const float a, float&s, float&c) {
	s = sin(a);
	c = cos(a);
	/*
	const double ss = s*s;
	c = (float)sqrt(1 - fmin(1.0f, ss));
	//30.7.2017 corected
	if (fabs(a) > 90)
		c = -c;
		*/
}
int init_shmPTR();
void close_shmPTR();

#endif


