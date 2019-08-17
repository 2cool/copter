#pragma once




#include "define.h"

#include "helper_3dmath.h"

class MadgwickAHRS
{


private:

	void MadgwickAHRSupdateIMU(Quaternion_ &q, float gx, float gy, float gz, float ax, float ay, float az, float dt);
	

public:



	//---------------------------------------------------------------------------------------------------
	// Variable definitions

	
	void setBeta(float b);
	float invSqrt(float x);
	void MadgwickAHRSupdate(Quaternion_ &q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
};

extern MadgwickAHRS AHRS;