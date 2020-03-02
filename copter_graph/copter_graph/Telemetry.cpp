#include "stdafx.h"
#include "Telemetry.h"


Telemetry::Telemetry()
{
}


Telemetry::~Telemetry()
{
}


uint16_t data[5];

void Telemetry::parser(byte buf[], int n, int cont_bits, bool filter, bool rotate) {

	memcpy((byte*)data, &buf[n], 10);



	const float hall_effect_sensor_max_cur[4] = { 36,37,43,34 };
	static float data_at_zero[4] = { 3255,3255,3255,3255 };
	static float cur_k[4] = { 0.011,0.011,0.011,0.011 };
	if ((cont_bits & 1) == 0) {
		for (int i = 0; i < 4; i++) {
			data_at_zero[i] += (((float)data[i]) - data_at_zero[i]) * 0.01f;
			cur_k[i] = hall_effect_sensor_max_cur[i] / data_at_zero[i];
		}
	}

	m_current[0] = abs(hall_effect_sensor_max_cur[0] - ((float)data[0]) * cur_k[0]);
	m_current[1] = abs(hall_effect_sensor_max_cur[1] - ((float)data[1]) * cur_k[1]);
	m_current[2] = abs(hall_effect_sensor_max_cur[2] - ((float)data[2]) * cur_k[2]);
	m_current[3] = abs(hall_effect_sensor_max_cur[3] - ((float)data[3]) * cur_k[3]);
	voltage = 0.564 * (float)(data[4]);

}

	Telemetry tel;