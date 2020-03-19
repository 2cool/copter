#pragma once
#include <stdint.h>
#include "helper_3dmath.h"
#include "MadgwickAHRS.h"
#include "graph.h"
class commander
{

public:
	float throttle, yaw, yaw_offset, pitch, roll;
	void parser(byte buf[], int j, int len, int cont_bits, bool filter, bool rotate);
};

extern commander com;