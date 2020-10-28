// 
// 
// 

//тайминг виставлять от начала полета. типа висеть тут 60 сек от начала. тоесть елс иприлетел в 59 сек то висеть сек.  ???
//добавить слежение за точкой.

//писал что батареи недостаточно чтоби пролететь 300 метров

#include "Prog.h"
#include "GPS.h"
#include "Autopilot.h"
#include "Stabilization.h"
#include "debug.h"
#include "Telemetry.h"

enum { LAT_LON = 1, DIRECTION = 2, ALTITUDE = 4,  CAMERA_ANGLE = 8, TIMER = 16,SPEED_XY=32,SPEED_Z=64,DO_ACTION=128};



//int high 4bits ZOOM
enum {LED0,LED1,LED2,LED3,LED4,LED5,LED6,PHOTO,START_VIDEO,STOP_VIDEO, PHOTO360, DO_NOTHIN};


#define MAX_HA 3
#define MAX_VA 1

void ProgClass::init(){
	Autopilot.program_is_loaded(false);
	intersactionFlag = false;
	speed_X = speed_Y = speed_Z = 0;
	steps_count = 0;
	step_index=0;
	prog_steps_count_must_be = 0;
	prog_data_index = 0;
	prog_data_size = 0;

}

/*

shmPTR->fpv_adr = *(buf+i++);
shmPTR->fpv_port = *(int16_t*)(buf + i);
i += 2;
shmPTR->fpv_zoom = *(buf + i++);
shmPTR->fpv_code = *(int16_t*)(buf + i);

*/



int32_t zoom__time=0;
uint8_t zoom_cam, old_zoom = -1, old_zoom1 = -1;

bool zoom_control() {
	if (zoom__time > 0 && millis_() < zoom__time)
		return true;
	if (zoom_cam != old_zoom1) {
		shmPTR->fpv_zoom = zoom_cam+1;
		shmPTR->fpv_code = 0;
		cout << "do_zoom "<< (int)zoom_cam<<endl;
		old_zoom1 = zoom_cam;
		zoom__time = millis_() + 5e3;
		return true;
	}
	zoom__time = 0;
	return false;
}
bool ProgClass::do_cam_action(const uint16_t code) {
	if (zoom_control() == false) {
		shmPTR->fpv_code = code;
		cout << "do_action " << code << endl;
		return false;
	}
	else
		return true;
}
bool ProgClass::takePhoto() { return do_cam_action(769); }
	
bool ProgClass::startVideo() {return do_cam_action(513);}
bool ProgClass::stopVideo()  {return do_cam_action(514);}



void ProgClass::cameraZoom() { shmPTR->fpv_code = zoom_cam; }


void ProgClass::takePhoto360() {
	static int32_t cam__time = 0;
	static double cam_yaw = 0,cam_pitch = 0;
	const int32_t _ct = millis_();
	if (cam__time == 0) {
		cam__time = _ct + 6e3;
		cam_yaw = cam_pitch = 0;
		Autopilot.setYaw(cam_yaw);
		Autopilot.set_gimBalPitch(cam_pitch);
		return;
	}
	if (_ct >= cam__time) {
		if (takePhoto() == false) {
			Autopilot.setYaw(cam_yaw);
			cam__time = _ct + 4e3;
			cam_yaw += 22.5;
			if (cam_yaw > 360) {
				if (cam_pitch == 0) {
					cam_yaw = 0;
					cam_pitch += 30;
					Autopilot.setYaw(cam_yaw);
					Autopilot.set_gimBalPitch(cam_pitch);
					cam__time = _ct + 6e3;
				}
				else {
					cam__time = 0;
					do_action = false;
				}
			}
		}
	}

}
void ProgClass::Do_Action() {

		
	if (action <= LED6)
		do_action = false;
	else 
		switch (action) {
			case PHOTO:
				do_action=takePhoto();
				break;
			case START_VIDEO:
				do_action=startVideo();
				break;
			case STOP_VIDEO:
				do_action = stopVideo();
				break;
			case PHOTO360:
				takePhoto360();
				break;
			default:
				do_action = false;
	}
}

#define MIN_DT 0.01

//камеоа млєеь сдежиь за точкой в пространстве. которую ей надо передать
//все делать через таймер. через время за которое надо єто зделать.


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ProgClass::loop(){
	const int32_t _ct = millis_();
	double dt = 0.001*(_ct - begin_time);
	
	if (do_action)
		Do_Action();

	if (go_next == false) {

		if (timer == 0) {
			if (altFlag == false)
				altFlag = (alt == old_alt) || (fabs(Mpu.get_Est_Alt() - Autopilot.fly_at_altitude()) <= (ACCURACY_Z));

			if (distFlag == false) {
				if (next_y == old_y && next_x == old_x) {
					distFlag = true;
				}
				else {
					const double advance_dist = Stabilization.getDist_XY(max_speed_xy);
					const double acur = fmax(fmax(ACCURACY_XY, GPS.loc.accuracy_hor_pos_), advance_dist);
					distFlag = Stabilization.get_dist2goal() <= acur;
				}
			}
			go_next = altFlag & distFlag;
		}
		else {
			if (timer <= dt)
				go_next = true;

		}
	}
	if (go_next && !do_action){
		go_next = distFlag = altFlag = do_action = false;
		if (load_next(true) == false){
			cout << "PROG END" << "\t"<<millis_() << endl;
			Autopilot.start_stop_program(false);
		}
	}
	intersactionFlag = Prog.getIntersection(need_X, need_Y);
}


#define MAX_TIME_LONG_FLIGHT  1200
bool ProgClass::program_is_OK(){
	double timeLeft=Telemetry.fly_time_left();

	if (prog_data_size >= 14 && prog_steps_count_must_be == steps_count){
		prog_data_index = 0;
		time4step2done = 0;
		begin_time = 0;
		next_x = GPS.loc.dX;
		next_y = GPS.loc.dY;
		alt = Mpu.get_Est_Alt();
		uint8_t step = 1;
		double fullTime = 0;
		while (load_next(false)){
			
			if (next_y != old_y && next_x != old_x){
				const double dx = next_x - old_x;
				const double dy = next_y - old_y;
				double time = (double)(sqrt(dx*dx + dy*dy) / max_speed_xy);
				const double dAlt = alt - old_alt;
				time += dAlt / ((dAlt >= 0) ? max_speedZ_P : max_speedZ_M);
				time *= 1.25f;
				time += timer;
				
				fullTime += time;
				if (fullTime>timeLeft){//MAX_TIME_LONG_FLIGHT){
					cout << "to long fly for prog!" <<" fly time="<<fullTime<<". Time left="<<timeLeft<<".\t"<<millis_() << endl;
					Telemetry.addMessage(e_PROG_TOO_LONG_DISTANCE);
					return false;
				}
			old_y = next_y;
			old_x = next_x;
			}
			step++;

			old_alt = alt;

		}
		const double x2 = next_x - GPS.loc.dX;
		const double y2 = next_y - GPS.loc.dY;
		const double dist = sqrt(x2*x2 + y2*y2);
		if (dist >= 20 || alt  >= 20){
			cout << "end poitn to far from star!!!" << "\t"<<millis_() << endl;
			Telemetry.addMessage(e_PROG_TOO_LONG_FROM_START);
			return false;
		}
		cout << "time for flyghy: " << (int)fullTime << "\t"<<millis_() << endl;
		return true;
	}
else
	return false;

}

bool ProgClass::start(){
	old_zoom1=old_zoom = -1;
	if (Autopilot.program_is_loaded() && program_is_OK()){
		old_zoom1 = old_zoom = -1;
		step_index = 0;
		prog_data_index = 0;
		time4step2done = 0;
		begin_time = 0;
		next_x = GPS.loc.dX;
		next_y = GPS.loc.dY;
		alt = Mpu.get_Est_Alt();
		go_next = distFlag = altFlag = do_action = true;
		action = LED0;
		return true;
	}
	else{
		clear();
		cout << "no program\n";
	}
	return false;

}

#define SGN(x) ((x<0)?-1:1)
//double sgn(const double x){ return (x < 0) ? -1 : 1; }

bool ProgClass::getIntersection(double &x, double &y){
	if (next_x == old_x && next_y == old_y){
		//ErrorLog.println("len=0");
		return false;
	}

	//double ks = 1;

	const double dist_ = Stabilization.get_dist2goal();

	double r = Stabilization.getDist_XY(max_speed_xy + Stabilization.get_allowance());
	if (r > dist_){
		//ErrorLog.println("r>dist");
		return false;
	}
	//----------------------------

	const double x2 = next_x - GPS.loc.dX;// GPS.loc.from_lat2X((double)(lat - GPS.loc.lat_));
	const double x1 = old_x - GPS.loc.dX;// GPS.loc.from_lat2X((double)(old_lat - GPS.loc.lat_));
	const double y2 = next_y - GPS.loc.dY;// GPS.loc.form_lon2Y((double)(lon - GPS.loc.lon_));
	const double y1 = old_y - GPS.loc.dY;// GPS.loc.form_lon2Y((double)(old_lon - GPS.loc.lon_));
	const double dx = x2 - x1;
	const double dy = y2 - y1;
	const double l2 = dx*dx + dy*dy;
	//const double dr = sqrt(l2);
	const double D = x1*y2 - x2*y1;

	double discriminant = (r*r*l2) - (D*D);
	if (discriminant <= 0){
		//ErrorLog.println("dis<0");
    // нахождение от точки до прямой.
		{
			const double dot = -x1 * dx - y1 * dy;
			double param = -1;
			if (l2 == 0) //in case of 0 length line
				return false;

			param = dot / l2;
			double xx, yy;

			if (param < 0) {
				xx = x1;
				yy = y1;
			}
			else if (param > 1) {
				xx = x2;
				yy = y2;
			}
			else {
				xx = x1 + param * dx;
				yy = y1 + param * dy;
			}
			double dist2line;


			if ((x2 - xx)*(xx - x1)<0 || (y2 - yy)*(yy - y1)<0){//точка не на линии
				//ErrorLog.println("not on line");
				double dx = x2 - xx;
				double dy = y2 - yy;
				const double dist2 = dx*dx+dy*dy;
				dx = x1 - xx;
				dy = y1 - yy;
				const double dist1 = dx*dx + dy*dy;
				dist2line = (1.0 + sqrt(fmin(dist2, dist1)));
			}else
				dist2line = (1.0 + sqrt(xx * xx + yy * yy));

			if (dist2line > r){
				if (dist2line > MAX_DIST_ERROR_TO_ACT){
					Telemetry.addMessage(e_TOO_STRONG_WIND);
					Autopilot.going2HomeStartStop(false);
					return true;
				}
				//ks = r / dist2line;
			}
			r = dist2line;
			discriminant = (r*r*l2) - (D*D);
		}
	//---------------------------------
	}
	if (discriminant <= 0)
		return false;
	discriminant = sqrt(discriminant);
	const double rdr2 = 1.0f/l2;
	double temp = SGN(dy)*dx*discriminant;
	const double ix1 = (D*dy + temp)*rdr2;
	const double ix2 = (D*dy - temp)*rdr2;
	temp = fabs(dy)*discriminant;
	const double iy1 = (-D*dx + temp)*rdr2;
	const double iy2 = (-D*dx - temp)*rdr2;

	double tx = x2 - ix1;
	double ty = y2 - iy1;
	const double dist1 = tx*tx + ty*ty;

	tx = x2 - ix2;
	ty = y2 - iy2;
	const double dist2 = tx*tx + ty*ty;

	if (dist1<dist2){
		x = ix1;
		y = iy1;
	}
	else{
		x = ix2;
		y = iy2;
	}

	return true;
}


void ProgClass::clear(){
	init();
}

bool ProgClass::load_next(bool loadf) {
	old_x = next_x;
	old_y = next_y;
	old_alt = alt;
	if (prog_data_index >= prog_data_size || steps_count <= step_index) {
		return false;
	}
	step_index++;
	//	printf("\nNext\n");

	int wi = prog_data_index + 1;
	if (prog[prog_data_index] & TIMER) {
		timer = prog[wi++];
	}
	

	if (prog[prog_data_index] & SPEED_XY) {
		max_speed_xy = fabs(prog[wi++]);
		Stabilization.set_max_speed_hor(max_speed_xy,!loadf);
	}
	
	if (prog[prog_data_index] & SPEED_Z) {
		float speedZ = fabs(prog[wi++]);
		max_speedZ_P = speedZ;
		max_speedZ_M = -speedZ;
		Stabilization.set_max_sped_ver(max_speedZ_P, max_speedZ_M, !loadf);	
	}

	if (prog[prog_data_index] & LAT_LON){
		int32_t lat, lon;
		*((uint32_t*)&lat) = *((uint32_t*)&prog[wi]); wi += 4;
		*((uint32_t*)&lon) = *((uint32_t*)&prog[wi]); wi += 4;
		if (loadf) 
			Stabilization.setNeedLoc(lat, lon, next_x, next_y);
		else
			Stabilization.fromLoc2Pos(lat, lon, next_x, next_y);
	}


	if (prog[prog_data_index] & DIRECTION){
		oldDir = 1.4173228*(double)prog[wi++];
		if (loadf)
			Autopilot.setYaw(-oldDir);
	}

	if (prog[prog_data_index] & ALTITUDE){
		int16_t ialt;
		byte*lb = (byte*)&ialt;
		lb[0] = prog[wi++];
		lb[1] = prog[wi++];
		alt = ialt;
		if (loadf)
			Autopilot.set_new_altitude(alt);
	}


	if (prog[prog_data_index] & CAMERA_ANGLE){
		old_cam_angle = 1.4173228f*(int8_t)prog[wi++];
		//printf("camera ang=%f\n", old_cam_angle);
		if (loadf)
			Autopilot.set_gimBalPitch(old_cam_angle);

	}

#define TIME2TEKE_PHOTO360 160
#define TIME2TEKE_PHOTO_OR_START_VIDEO 2
#define TIME4ZOOM 5

	if (prog[prog_data_index] & DO_ACTION) {
		action = prog[wi++];
		do_action =  !(action == DO_NOTHIN);
		if (do_action) {
			if (action >= PHOTO && action <= PHOTO360) {
				zoom_cam = prog[wi++];
				cout << "zoom_loaded=" << (int)zoom_cam << endl;
				if (timer < TIME2TEKE_PHOTO_OR_START_VIDEO)
					timer = TIME2TEKE_PHOTO_OR_START_VIDEO;
				
				if (zoom_cam != old_zoom) {
					old_zoom = zoom_cam;
					if (timer < TIME4ZOOM)
						timer = TIME4ZOOM;
				}
			}
			if (!loadf && action == PHOTO360 && timer < TIME2TEKE_PHOTO360) {
				timer = TIME2TEKE_PHOTO360;
			}
			
		}
		else
			do_action == false; // temp//not used

		do_action &= loadf;
		
	}

	prog_data_index = wi;
	begin_time = millis_();
	need_X = need_Y = 0;
	return true;

}




bool ProgClass::add(byte*buf)
{
	uint16_t pi = prog_data_size;
	uint8_t i = 1;
	prog[pi++] = buf[0];
	//printf("mask= %i\n",buf[0]);
	
	if (steps_count != buf[i++]){
		clear();
		Telemetry.addMessage(e_PROG_INDEX_ERROR); 
		return false;
	}
	if (i + 17 > PROG_MEMORY_SIZE){
		clear();
		cout << "PROG_MEMORY_OVERFLOW\n";
		return false;
	}
//	Out.printf("mask:"); Out.println(buf[0]);
	
	if (buf[0] & TIMER){
		prog[pi++] = buf[i++];
		//cout << "timer " << (int)buf[i - 1] << endl;
	}

	if (buf[0] & SPEED_XY){
		prog[pi++] = buf[i++];
		//cout << "speedXY "<<(int)buf[i - 1] << endl;
	}
	if (buf[0] & SPEED_Z){
		prog[pi++] = buf[i++];
		//cout <<"speedZ "<< (int)buf[i - 1] << endl;
	}

	if (buf[0] & LAT_LON){
		*(uint64_t*)&prog[pi] = *(uint64_t*)&buf[i];
		//cout << "LAT " << *(int32_t*)& prog[pi] << endl;
		//cout << "LON " << *(int32_t*)& prog[pi+4] << endl;
		pi += 8;
		i += 8;
	}
	

	if (buf[0] & DIRECTION){
		prog[pi++] = buf[i++];
		//cout << "dirrection " << (int)buf[i - 1] << endl;
	}
	

	if (buf[0] & ALTITUDE){
		*(uint16_t*)&prog[pi] = *(uint16_t*)&buf[i];
		pi += 2;
		i += 2;
		//cout << "altitude " << (int) *(uint16_t*)& buf[i - 2] << endl;

	}

	if (buf[0] & CAMERA_ANGLE){
		prog[pi++] = buf[i++];
		//cout << "cam_angle " << (int)buf[i - 1] << endl;
	}

	if (buf[0] & DO_ACTION){
		prog[pi] = buf[i++];
		if (prog[pi] >= PHOTO && prog[pi] <= PHOTO360) {
			pi++;
			prog[pi] = buf[i++];
			cout << "= zoom =" << (int)prog[pi] << endl;
		}
		pi++;
			
		//cout << "action " << (int)buf[i - 1] << endl;
	}

	if (steps_count == 0){
		prog_steps_count_must_be = *(uint16_t*)&buf[i];
		i+=2;
		//cout << "prog steps=" << prog_steps_count_must_be << "\t"<<millis_() << endl;
	}
	
	prog_data_size = pi;
	steps_count++;
	cout << steps_count << ". dot added! " << prog_data_size << "\t"<<millis_() << endl;
	//Autopilot.program_is_loaded(prog_steps_count_must_be == steps_count);
	if (prog_steps_count_must_be == steps_count) {
		Autopilot.program_is_loaded(true);
	}
	return true;
}

ProgClass Prog;

