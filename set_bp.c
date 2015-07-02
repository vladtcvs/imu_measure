#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>

int main(int argc, char **argv)
{
	uint8_t regs[256];
	uint8_t addr = 0x68;
	int bp = 0;
	int reg;

	if (argc != 2)	
		return 0;
	sscanf(argv[1], "%i", &bp);
	int fd = open("/dev/i2c-2", O_RDWR);
	if (fd < 0) {
		printf("can not open\n");
		return 0;
	}
	
	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		close(fd);
		printf("can not ioctl\n");
		return 0;
	}

	uint8_t r = 55, d;
	write(fd, &r, 1);
	read(fd, &d, 1);
	if (bp) {
		d |= 0x02;
	} else {
		d &= 0xFD;
	}
	uint8_t buf[2] = {r, d};
	write(fd, buf, 2);

	if (bp) {
		addr = 0x0c;	
		if (ioctl(fd, I2C_SLAVE, addr) < 0) {
			close(fd);
			printf("can not ioctl\n");
			return 0;
		}
		buf[0] = 0x0A;
		buf[1] = 0x12;
		write(fd, buf, 2);
	}
	close(fd);

	return 0;
}

