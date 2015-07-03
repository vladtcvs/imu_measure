#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <mpu_9250.h>
#include <stdio.h>

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
	uint8_t regs[256];
	
	int reg;
	
	if (data == NULL)
		return -2;

	if (ioctl(fd, I2C_SLAVE, addr_ga) < 0)
		return -1;

	for (reg = 0; reg < 256; reg++) {
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

	for (reg = 0; reg < 256; reg++) {
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
	data->wx = raw.wx / gsens;
	data->wy = raw.wy / gsens;
	data->wz = raw.wz / gsens;

	data->ax = raw.ax / asens;
	data->ay = raw.ay / asens;
	data->az = raw.az / asens;

	data->temp = raw.temp / 100.0;
	return 0;
}

int setup_imu(int fd, int gfs, int afs)
{
	uint8_t buf[2];
	buf[0] = 27;
	gfs &= 0x03;
	buf[1] = gfs << 3;
	write(fd, buf, 2);
	switch (gfs) {
	case 0:
		gsens = 13100.;
		break;
	case 1:
		gsens = 6550.;
		break;
	case 2:
		gsens = 3280.;
		break;
	case 3:
		gsens = 1640.;
		break;
	}
	buf[0] = 28;
	afs &= 0x03;
	buf[1] = afs << 3;
	write(fd, buf, 2);
	switch (afs) {
	case 0:
		asens = 1638.4;
		break;
	case 1:
		asens = 819.2;
		break;
	case 2:
		asens = 409.6;
		break;
	case 3:
		asens = 204.8;
		break;
	}
}

