#include <set_pwm.h>
#include <mpu9250_unit.h>
#include <stabilization.h>
#include <stdio.h>

stabilizer::stabilizer(const char* devname, int speed, int gpiocs)
{
	if (engines.open_engines(devname, speed, gpiocs) <= 0) {
		printf("Failed to configure spi\n");
		return;
	}
	h1 = h2 = h3 = 1;
	r = 0;
	p = 0;
	y = 0;
}

void stabilizer::turn_off()
{
	double pwms[NCH] = {0};
	bool en[NCH] = {false};

	en[0] = en[1] = en[2] = en[3] = true;
	engines.set_pwm(pwms, en);
}

void stabilizer::current_rpy(float R, float P, float Y)
{
	double pwms[NCH];
	bool en[NCH] = {false};
	 // difference between target angle and current
	float dR = -(r - R);
	float dP = -(p - P);
	float dY = -(y - Y);

	if (dY > M_PI)
		dY -= 2*M_PI;
	if (dY < -M_PI)
		dY += 2*M_PI;

	float f1, f2, f3, f4;
	f1 = 0.25 * (h1 * dR - h2 * dP - h3 * dY + power);
	f2 = 0.25 * (-h1 * dR - h2 * dP + h3 * dY + power);
	f3 = 0.25 * (-h1 * dR + h2 * dP - h3 * dY + power);
	f4 = 0.25 * (h1 * dR + h2 * dP + h3 * dY + power);

	if (f1 < 0)
		f1 = 0;
	if (f2 < 0)
		f2 = 0;
	if (f3 < 0)
		f3 = 0;
	if (f4 < 0)
		f4 = 0;

	en[0] = en[1] = en[2] = en[3] = true;
	pwms[0] = f1;
	pwms[1] = f2;
	pwms[2] = f3;
	pwms[3] = f4;
	engines.set_pwm(pwms, en);
}

void stabilizer::set_parameters(float Power, float R, float P, float Y)
{
	power = Power;
	r = R;
	p = P;
	y = Y;
}
