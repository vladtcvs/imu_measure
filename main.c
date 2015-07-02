#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>

#include <mpu_9250.h>

int main(void)
{
	orient_data_t data;
	int fd = open_imu("/dev/i2c-2");
	if (fd == 0)
		return 0;
	setup_imu(fd, 0, 1);
	while (1) {
		if (read_imu(fd, &data) >= 0) {
			printf("wx = %lf\n", data.wx);	
			printf("wy = %lf\n", data.wy);
			printf("wz = %lf\n\n", data.wz);
			printf("ax = %lf\n", data.ax);	
			printf("ay = %lf\n", data.ay);
			printf("az = %lf\n", data.az);
			printf("\n\n");
		}
		sleep(1);
	}
	close_imu(fd);
	return 0;
}

