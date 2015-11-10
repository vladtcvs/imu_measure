#ifndef __SET_PWM_H__
#define __SET_PWM_H__

#include <stdint.h>
#define NCH 11

class set_engines {
	const uint8_t MAX_PWM;
	int fd;
	int gpio_cs;
	int speed;
	int bits;
	int delay;
	int transfer(uint8_t *tx, uint8_t *rx, int num);
	uint8_t pwms[NCH];
public:
	set_engines();
	~set_engines();
	int open_engines(const char *device, int speed, int gpio_cs);
	int close_engines();
	int set_pwm(int id, double val);
	int set_pwm(double vals[NCH]);
};

#endif
