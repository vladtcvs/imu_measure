#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <mpu_9250.h>
#include <stdio.h>
#include <math.h>

const static uint8_t addr_ga = 0x68;
const static uint8_t addr_m = 0x0c;
static double gsens = 131.0, asens = 2.048;

int open_imu(const char *devname)
{
	int fd = open(devname, O_RDWR);

	if (fd < 0) {
		printf("can not open\n");
		return 0;
	}
	return fd;
}

void close_imu(int fd)
{
	close(fd);
}

int enable_compass(int fd, int enable)
{
	if (ioctl(fd, I2C_SLAVE, addr_ga) < 0) {
		close(fd);
		printf("can not ioctl\n");
		return -1;
	}
	uint8_t r = 55, d;
	write(fd, &r, 1);
	read(fd, &d, 1);
	if (enable) {
		d |= 0x02;
	} else {
		d &= 0xFD;
	}
	uint8_t buf[2] = {r, d};
	write(fd, buf, 2);
	if (enable) {
		if (ioctl(fd, I2C_SLAVE, addr_m) < 0) {
			printf("can not ioctl\n");
			return -1;
		}
		buf[0] = 0x0A;
		buf[1] = 0x12;
		write(fd, buf, 2);
	}
	return 0;
}

typedef union {
	struct {
		int16_t ax, ay, az;
		int16_t wx, wy, wz;
		int16_t mx, my, mz;
		int16_t temp;
	};
	int16_t data[10];
} imu_data_t;

int read_imu_raw(int fd, imu_data_t *data)
{
	static uint8_t regs[256];
	
	int reg;
	
	if (data == NULL)
		return -2;

	if (ioctl(fd, I2C_SLAVE, addr_ga) < 0)
		return -1;

	for (reg = 59; reg <= 72; reg++) {
		uint8_t r = reg;
		int res;
		write(fd, &r, 1);
		res = read(fd, &(regs[reg]), 1);
	}

	data->ax = 0;
	data->ax |= regs[59];
	data->ax <<= 8;
	data->ax |= regs[60];

	data->ay = 0;
	data->ay |= regs[61];
	data->ay <<= 8;
	data->ay |= regs[62];

	data->az = 0;
	data->az |= regs[63];
	data->az <<= 8;
	data->az |= regs[64];

	data->temp = 0;
	data->temp |= regs[65];
	data->temp <<= 8;
	data->temp |= regs[66];

	data->wx = 0;
	data->wx |= regs[67];
	data->wx <<= 8;
	data->wx |= regs[68];

	data->wy = 0;
	data->wy |= regs[69];
	data->wy <<= 8;
	data->wy |= regs[70];
	
	data->wz = 0;
	data->wz |= regs[71];
	data->wz <<= 8;
	data->wz |= regs[72];

	if (ioctl(fd, I2C_SLAVE, addr_m) < 0)
		return -1;

	for (reg = 3; reg <= 8; reg++) {
		uint8_t r = reg;
		int res;
		write(fd, &r, 1);
		res = read(fd, &(regs[reg]), 1);
	}

	data->mx = 0;
	data->mx |= regs[4];
	data->mx <<= 8;
	data->mx |= regs[3];

	data->my = 0;
	data->my |= regs[6];
	data->my <<= 8;
	data->my |= regs[5];
	
	data->mz = 0;
	data->mz |= regs[8];
	data->mz <<= 8;
	data->mz |= regs[7];
	return 0;
}

int read_imu(int fd, orient_data_t *data)
{
	imu_data_t raw;
	if (read_imu_raw(fd, &raw) < 0)
		return -1;
	data->wx = raw.wx / gsens * M_PI/180;
	data->wy = raw.wy / gsens * M_PI/180;
	data->wz = raw.wz / gsens * M_PI/180;

	data->ax = raw.ax / asens;
	data->ay = raw.ay / asens;
	data->az = raw.az / asens;

	/* compass has different orientation */
	data->mx = raw.my;
	data->my = raw.mx;
	data->mz = -raw.mz;

	data->temp = raw.temp / 100.0;
	return 0;
}

int setup_imu(int fd, enum mpu9250_gfs_e gfs, enum mpu9250_afs_e afs)
{
	uint8_t fchoice = 0;
	uint8_t buf[2];

	/* setup gyro */
	buf[0] = 27;
	buf[1] = 0;
	buf[1] |= fchoice & 0x03;

	switch (gfs) {
	case GFS_250_DPS:
		buf[1] |= 0x00 << 3;
		gsens = 32768/250.0;
		break;
	case GFS_500_DPS:
		buf[1] |= 0x01 << 3;
		gsens = 32768/500.0;
		break;
	case GFS_1000_DPS:
		buf[1] |= 0x10 << 3;
		gsens = 32768/1000.0;
		break;
	case GFS_2000_DPS:
		buf[1] |= 0x11 << 3;
		gsens = 32768/2000.0;
		break;
	}
	write(fd, buf, 2);

	/* setum accelerometer */
	buf[0] = 28;
	buf[1] = 0;

	switch (afs) {
	case AFS_2G:
		buf[1] |= 0x00 << 3;
		asens = 32768/(2*9.81);
		break;
	case AFS_4G:
		buf[1] |= 0x01 << 3;
		asens = 32768/(4*9.81);
		break;
	case AFS_8G:
		buf[1] |= 0x10 << 3;
		asens = 32768/(8*9.81);
		break;
	case AFS_16G:
		buf[1] |= 0x11 << 3;
		asens = 32768/(16*9.81);
		break;
	}
	write(fd, buf, 2);

	buf[0] = 29;
	buf[1] |= 1 << 3;
	buf[1] |= 0x00 & 0x07;
	write(fd, buf, 2);
}
