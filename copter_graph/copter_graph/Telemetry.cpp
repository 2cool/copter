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
	 m_current[0] = 1.024 * (20 - (float)(data[0] - 24) / CUR_K);
	m_current[1] = 1.024 * (20 - (float)(data[1] - 24) / CUR_K);
	m_current[2] = 1.024 * (20 - (float)(data[2] - 24) / CUR_K);
	m_current[3] = 1.024 * (20 - (float)(data[3] - 24) / CUR_K);





	voltage = 1.725*(float)(data[4]);

}

	Telemetry tel;