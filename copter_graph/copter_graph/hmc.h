#pragma once
#include "graph.h"
class Hmc
{
public:

	float heading=0;
	void parser(byte buf[], int n);


	Hmc();
	~Hmc();
};
extern Hmc hmc;

