#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <mpu_9250.h>
#include <sys/time.h>

struct timeval tv1,tv2,dtv;
struct timezone tz;

void time_start(void)
{
	gettimeofday(&tv1, &tz);
}

long time_stop(void)
{
	gettimeofday(&tv2, &tz);
	dtv.tv_sec = tv2.tv_sec -tv1.tv_sec;
	dtv.tv_usec = tv2.tv_usec - tv1.tv_usec;
	if (dtv.tv_usec < 0) {
		dtv.tv_sec--;
		dtv.tv_usec += 1000000;
	}
	return dtv.tv_sec*1000 + dtv.tv_usec/1000;
}

int main(void)
{
	orient_data_t data;
	double t;
	int fd = open_imu("/dev/i2c-2");
	if (fd == 0)
		return 0;
	enable_compass(fd, 1);
	setup_imu(fd, 0, 1);
	time_start();
	while (1) {
		t = time_stop()/1000.0;

		if (read_imu(fd, &data) >= 0) {
			printf("%lf %lf %lf %lf\n", 
			       t, data.wx, data.wy, data.wz);
		}
		//usleep(10000);
	}
	close_imu(fd);
	return 0;
}

