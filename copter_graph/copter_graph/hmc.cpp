#include "stdafx.h"
#include "hmc.h"


Hmc::Hmc()
{
}


Hmc::~Hmc()
{
}

//enum{X,Y,Z};

uint8_t buffer[6];
int16_t mx, my, mz,c_base[3];
float fmx, fmy, fmz,dx,dy,dz;
void Hmc::parser(byte buf[], int n) {

	heading = *(float*)&buf[n];

}
Hmc hmc;