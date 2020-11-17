#pragma once
#include "graph.h"
class Hmc
{
public:

	float heading=0;
	void parser(byte buf[], int n);
	void parser_sens(byte buf[], int n, int cont_bits, bool filter, bool rotate);
	void parser_base(byte buf[], int n);
	float fmx, fmy, fmz;
	float yaw_correction_angle = 0;

	Hmc();
	~Hmc();
};
extern Hmc hmc;

