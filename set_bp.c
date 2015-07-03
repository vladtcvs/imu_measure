#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>
#include <mpu_9250.h>

int main(int argc, char **argv)
{
	uint8_t regs[256];
	uint8_t addr = 0x68;
	int bp = 0;
	int reg;

	if (argc != 2)	
		return 0;
	sscanf(argv[1], "%i", &bp);
	int fd=open_imu("/dev/i2c-2");
	if (fd == 0)
		return 0;
	enable_compass(fd, bp);
	close_imu(fd);
	return 0;
}

