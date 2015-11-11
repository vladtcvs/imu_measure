#include <set_pwm.h>
#include <mpu9250_unit.h>
#include <stabilization.h>
#include <stdio.h>

stabilizer::stabilizer(const char* devname, int speed, int gpiocs)
{
	engines.open_engines(devname, speed, gpiocs);
	h1 = h2 = h3 = 1;
	r = 0;
	p = 0;
	y = 0;
}

void stabilizer::current_rpy(float R, float P, float Y)
{
	double pwms[NCH];
	float dR = r - R;
	float dP = p - P;
	float dY = y - Y;

	if (dY > M_PI)
		dY -= 2*M_PI;
	if (dY < -M_PI)
		dY += 2*M_PI;

	float f1, f2, f3, f4;
	f1 = 0.25 * (h1 * dR - h2 * dP + h3 * dY + power);
	f2 = 0.25 * (h1 * dR + h2 * dP - h3 * dY + power);
	f3 = 0.25 * (-h1 * dR + h2 * dP + h3 * dY + power);
	f4 = 0.25 * (-h1 * dR - h2 * dP - h3 * dY + power);

	if (f1 < 0)
		f1 = 0;
	if (f2 < 0)
		f2 = 0;
	if (f3 < 0)
		f3 = 0;
	if (f4 < 0)
		f4 = 0;

	pwms[0] = f1;
	pwms[1] = f2;
	pwms[2] = f3;
	pwms[3] = f4;
	//engines.set_pwm(pwms);
	printf("pwm1 = %f pwm2 = %f pwm3 = %f pwm4 = %f\n",
	       pwms[0], pwms[1], pwms[2], pwms[3]);
}

void stabilizer::set_parameters(float Power, float R, float P, float Y)
{
	power = Power;
	r = R;
	p = P;
	y = Y;
}
