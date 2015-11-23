#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <set_pwm.h>

set_engines::set_engines() :
			MAX_PWM(125)
{
	for (int i = 0; i < NCH; i++)
		pwms[i] = 0;
	bits = 8;
	delay = 0;
}

set_engines::~set_engines()
{
	if (fd) {
		//close_engines();
	}
	fd = 0;
}

int set_engines::open_engines(const char *device, int nspeed, int ngpio_cs)
{
	fd =  open(device, O_RDWR);
	speed = nspeed;
	gpio_cs = ngpio_cs;
	if (fd > 0) {
		uint32_t mode = 0;
		int gpio_fd;
		int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
		if (ret == -1) {
			close(fd);
			return ret;
		}

		ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
		if (ret == -1) {
			close(fd);
			return ret;
		}

		ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
		if (ret == -1) {
			close(fd);
			return ret;
		}

		gpio_fd = open("/sys/class/gpio/export", O_WRONLY);
		if (gpio_fd <= 0) {
			close(fd);
			return 0;
		}
		char buf[201];
		snprintf(buf, 200, "%d", gpio_cs);
		buf[200] = 0;
		write(gpio_fd, buf, strlen(buf) + 1);
		close(gpio_fd);

		snprintf(buf, 200, "/sys/class/gpio/gpio%d/direction", gpio_cs);
		buf[200] = 0;
		gpio_fd = open(buf, O_WRONLY);
		if (gpio_fd <= 0) {
			close(fd);
			return 0;
		}
		write(gpio_fd, "out", sizeof("out"));
		close(gpio_fd);

		snprintf(buf, 200, "/sys/class/gpio/gpio%d/value", gpio_cs);
		buf[200] = 0;
		gpio_fd = open(buf, O_WRONLY);
		if (gpio_fd <= 0) {
			close(fd);
			return 0;
		}
		write(gpio_fd, "1", sizeof("1"));
		close(gpio_fd);

		double tpwms[NCH];
		bool enable[NCH];
		for (int i = 0; i < NCH; i++) {
			tpwms[i] = 0;
			enable[i] = false;
		}
		enable[0] = enable[1] = enable[2] = enable[3] = true;
		set_pwm(tpwms, enable);
	}
	return fd;
}

int set_engines::close_engines()
{
	int gpio_fd;
	char buf[201];

	snprintf(buf, 200, "/sys/class/gpio/gpio%d/direction", gpio_cs);
	buf[200] = 0;
	gpio_fd = open(buf, O_WRONLY);
	if (gpio_fd <= 0) {
		close(fd);
		return -1;
	}
	write(gpio_fd, "in", sizeof("in"));
	close(gpio_fd);
	
	gpio_fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (gpio_fd <= 0) {
		close(fd);
		return -1;
	}
	snprintf(buf, 200, "%d", gpio_cs);
	buf[200] = 0;
	write(gpio_fd, buf, strlen(buf) + 1);
	close(gpio_fd);

	close(fd);
	return 0;
}

int set_engines::transfer(uint8_t *tx, uint8_t *rx, int num)
{
	int ret;
	struct spi_ioc_transfer tr;
	char buf[200];
	int gpio_fd;
	int i;

	/*printf("transfer: ");
	for (i = 0; i < num; i++)
		printf("%i ", tx[i]);
	printf("\n");*/

	snprintf(buf, 200, "/sys/class/gpio/gpio%d/value", gpio_cs);
	buf[200] = 0;
	gpio_fd = open(buf, O_WRONLY);
	if (gpio_fd <= 0) {
		close(fd);
		return 0;
	}
	write(gpio_fd, "0", sizeof("0"));
	close(gpio_fd);

	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;
	tr.len = num;
	tr.delay_usecs = delay;
	tr.speed_hz = speed;
	tr.bits_per_word = bits;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		return -1;

	gpio_fd = open(buf, O_WRONLY);
	if (gpio_fd <= 0) {
		close(fd);
		return 0;
	}
	write(gpio_fd, "1", sizeof("1"));
	close(gpio_fd);

	return 0;
}

int set_engines::set_pwm(int id, double val, bool en)
{
	int i;
	uint8_t rxbuf[NCH + 1], txbuf[NCH + 1];
	if (id < 0 || id >= NCH)
		return -1;
	if (val < 0)
		val = 0;
	if (val > 1)
		val = 1;
	if (en)
		pwms[id] = val * (MAX_PWM - 1) + 1;
	else
		pwms[id] = 0;

	for (i = 0; i < NCH; i++)
		txbuf[i + 1] = pwms[i];
	txbuf[0] = 0xFF;

	transfer(txbuf, rxbuf, NCH + 1);
	if (rxbuf[0] != 0xEE)
		return -2;
	return 0;
}

int set_engines::set_pwm(double vals[NCH], bool enable[NCH])
{
	int i;
	uint8_t rxbuf[NCH + 1], txbuf[NCH + 1];
	for (int i = 0; i < NCH; i++) {
		if (vals[i] < 0)
			vals[i] = 0;
		if (vals[i] > 1)
			vals[i] = 1;
		if (enable[i])
			pwms[i] = vals[i] * (MAX_PWM - 1) + 1;
		else
			pwms[i] = 0;
	}

	for (i = 0; i < NCH; i++)
		txbuf[i + 1] = pwms[i];
	txbuf[0] = 0xFF;

	transfer(txbuf, rxbuf, NCH + 1);
	if (rxbuf[0] != 0xEE)
		return -2;
	return 0;
}

int set_engines::set_pwm(double vals[NCH])
{
	uint8_t rxbuf[NCH + 1], txbuf[NCH + 1];
	int i;

	for (int i = 0; i < NCH; i++) {
		if (vals[i] < 0)
			vals[i] = 0;
		if (vals[i] > 1)
			vals[i] = 1;
		if (pwms[i] != 0)
			pwms[i] = vals[i] * (MAX_PWM - 1) + 1;
	}

	for (i = 0; i < NCH; i++)
		txbuf[i + 1] = pwms[i];
	txbuf[0] = 0xFF;

	transfer(txbuf, rxbuf, NCH + 1);
	if (rxbuf[0] != 0xEE)
		return -2;
	return 0;
}
