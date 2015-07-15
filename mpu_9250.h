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

enum mpu9250_gfs_e {
	GFS_250_DPS = 0,
	GFS_500_DPS = 1,
	GFS_1000_DPS = 2,
	GFS_2000_DPS = 3
};

enum mpu9250_afs_e {
	AFS_2G = 0,
	AFS_4G = 1,
	AFS_8G = 2,
	AFS_16G = 3
};

int enable_compass(int fd, int enable);
int setup_imu(int fd, enum mpu9250_gfs_e gfs, enum mpu9250_afs_e afs);
void close_imu(int fd);
int open_imu(const char *devname);
int read_imu(int fd, orient_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
