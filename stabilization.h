#pragma once

#include <set_pwm.h>

class stabilizer {
	float r, p, y;
	float Mx, My, Mz;
	float power;
	float h1, h2, h3;
	float pwm_0, pwm_1, pwm_2, pwm_3;
	set_engines engines;
public:
	stabilizer(const char *devname, int speed, int gpiocs);
	void set_parameters(float Power, float R, float P, float Y);
	void current_rpy(float R, float P, float Y);
	void set_hardness(float H1, float H2, float H3)
	{
		h1 = H1; h2 = H2; h3 = H3;
	}
	void turn_off();
};
