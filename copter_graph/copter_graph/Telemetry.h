#pragma once

#include "graph.h"
class Telemetry
{
public:
	float m_current[4], voltage;
	void parser(byte buf[], int n);
	Telemetry();
	~Telemetry();
};
extern Telemetry tel;
