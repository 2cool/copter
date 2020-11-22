#pragma once

#include "define.h"


class AutopilotClass
{
public:
	uint32_t control_bits;

	bool motors_is_on() { return control_bits & MOTORS_ON; }


};

extern AutopilotClass Autopilot;