// Prog.h

#ifndef _PROG_h
#define _PROG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "define.h"
#include "GeoDot.h"



class ProgClass
{
#define PROG_MEMORY_SIZE 20000
 protected:
	 bool go_next, distFlag, altFlag, do_action;
	 double next_x, next_y, old_x, old_y;
	 uint8_t action;

	 uint8_t oldTimer;
	 double old_alt, alt;
	 double oldDir, old_cam_angle;
	 uint16_t prog_data_size,prog_data_index;
	 uint16_t prog_steps_count_must_be;
	 uint8_t step_index,steps_count;
	 byte prog[PROG_MEMORY_SIZE];
	 double speed_X, speed_Y, speed_Z;
	 bool program_is_OK();
	 double r_time,time4step2done,speed_corected_delta;

	 int32_t begin_time;
	 double max_speed_xy,timer;
	 double max_speedZ_P, max_speedZ_M;
	 void takePhoto360();
	 bool takePhoto();
	 bool startVideo();
	 bool stopVideo();
	 void cameraZoom();
	 bool do_cam_action(const uint16_t code);
	 void Do_Action();
 public:
	 bool intersactionFlag;
	 double need_speedX, need_speedY;
	bool getIntersection(double &distX, double &distY);
	double altitude, direction, cam_pitch;
	 void init();
	// GeoDotClass gd;
	 bool add(byte*buf);
	 bool start();
	 bool load_next(bool);
	
	 void clear();
	 void loop();

	 void init(string buf){

	 }
};

extern ProgClass Prog;

#endif

