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



#define CUR_K ((float)HALL_EFFECT_SENSOR_MAX_CURRENT / 1024)
	const float addI = 0.5;
#define HALL_EFFECT_SENSOR_MAX_CURRENT 20
	m_current[0] += (addI + 1.25 * (HALL_EFFECT_SENSOR_MAX_CURRENT - (float)data[0] * CUR_K)-m_current[0])*0.03; //в лог пишем дата
	m_current[1] += (addI + 1.25 * (HALL_EFFECT_SENSOR_MAX_CURRENT - (float)data[1] * CUR_K)-m_current[1])*0.03;
	m_current[2] += (addI + 2 * (HALL_EFFECT_SENSOR_MAX_CURRENT - (float)data[2] * CUR_K)-m_current[2])*0.03;
	m_current[3] += (addI + 1.25 * (HALL_EFFECT_SENSOR_MAX_CURRENT - (float)data[3] * CUR_K)-m_current[3])*0.03;

	





	voltage = 1.725*(float)(data[4]);

}

	Telemetry tel;