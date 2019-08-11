#pragma once
#include "graph.h"
class Stab
{
public:
	Stab();
	~Stab();
	float sZ=0, speedZ=0, stab=0, fZ = 0;

	void parser(byte buf[], int n, int len);


};

extern Stab stab;