#ifndef __MPU_9250_H__
#define __MPU_9250_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
	double ax, ay, az;
	double wx, wy, wz;
	double mx, my, mz;
	double temp;
} orient_data_t;

int enable_compass(int fd, int enable);
int setup_imu(int fd, int gfs, int afs);
void close_imu(int fd);
int open_imu(const char *devname);
int read_imu(int fd, orient_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
