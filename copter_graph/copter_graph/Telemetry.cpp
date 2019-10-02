#include "stdafx.h"
#include "Telemetry.h"


Telemetry::Telemetry()
{
}


Telemetry::~Telemetry()
{
}


uint16_t data[5];
void Telemetry::parser(byte buf[], int n) {

	memcpy((byte*)data, &buf[n], 10);

#define CUR_K 51.15

	for (int i=0; i<4; i++)
		m_current[i] += (1.024 * (20 - (float)(data[i] - 24) / CUR_K) - m_current[i]) * 0.03;;





	voltage = 1.725*(float)(data[4]);

}

	Telemetry tel;