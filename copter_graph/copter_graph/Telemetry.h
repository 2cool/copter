#pragma once

#include "graph.h"
class Telemetry
{
public:
	float m_current[4], voltage;
	void parser(byte buf[], int n,int cont_bits, bool filter, bool rotate);
	Telemetry();
	~Telemetry();
};
extern Telemetry tel;
