#include "stdafx.h"
#include "Stab.h"


Stab::Stab()
{
}


Stab::~Stab()
{
}



void Stab::parser(byte buf[], int n, int len,const bool filtr) {


	sZ = *(float*)&buf[n];
	n += 4;
	speedZ=*(float*)&buf[n];
//	Log.loadFloat(stab);
//	Log.loadFloat(fZ);


	if (len == 16) {
		stab = *(float*)&buf[n];
		fZ =   *(float*)&buf[n];
	}


	

		
}









Stab stab;
