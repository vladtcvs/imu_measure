#include <mpu9250_unit.h>
#include <iostream>

mpu9250_unit::mpu9250_unit(std::string devname)
{
	offset.ax = 0;
	offset.ay = 0;
	offset.az = 0;

	offset.mx = 0;
	offset.my = 0;
	offset.mz = 0;

	offset.wx = 0;
	offset.wy = 0;
	offset.wz = 0;
	fd = open_imu(devname.c_str());
	if (fd == 0) {
		throw 1;
	}
	enable_compass(fd, 1);
	setup_imu(fd, GFS_250_DPS, AFS_2G);
}

mpu9250_unit::mpu9250_unit()
{
	offset.ax = 0;
	offset.ay = 0;
	offset.az = 0;

	offset.mx = 0;
	offset.my = 0;
	offset.mz = 0;

	offset.wx = 0;
	offset.wy = 0;
	offset.wz = 0;
	fd = open_imu("/dev/i2c-2");
	if (fd == 0) {
		throw 1;
	}
	enable_compass(fd, 1);
	setup_imu(fd, GFS_250_DPS, AFS_2G);
}

mpu9250_unit::~mpu9250_unit()
{
	if (fd)
		close_imu(fd);
}

bool mpu9250_unit::measure_offset(const int N)
{
	int i;
	int n = 0;
	offset.wx = 0;
	offset.wy = 0;
	offset.wz = 0;
	offset.ax = 0;
	offset.ay = 0;
	offset.az = 0;
	offset.mx = 0;
	offset.my = 0;
	offset.mz = 0;
	for (i = 0; i < N; i++) {
		orient_data_t dc;
		if (read_imu(fd, &dc) >= 0) {
			offset.wx += dc.wx;
			offset.wy += dc.wy;
			offset.wz += dc.wz;

			offset.ax += dc.ax;
			offset.ay += dc.ay;
			offset.az += dc.az;

			offset.mx += dc.mx;
			offset.my += dc.my;
			offset.mz += dc.mz;
			n++;
		}
	}
	if (n) {
		offset.wx /= n;
		offset.wy /= n;
		offset.wz /= n;

		offset.ax /= n;
		offset.ay /= n;
		offset.az /= n;

		offset.mx /= n;
		offset.my /= n;
		offset.mz /= n;
	} else {
		return false;
	}

	return true;
}


bool mpu9250_unit::measure()
{
	orient_data_t dc;
	if (read_imu(fd, &dc) < 0)
		return false;

	meas.ax = dc.ax;
	meas.ay = dc.ay;
	meas.az = dc.az;

	meas.wx = dc.wx - offset.wx;
	meas.wy = dc.wy - offset.wy;
	meas.wz = dc.wz - offset.wz;

	meas.mx = dc.mx;
	meas.my = dc.my;
	meas.mz = dc.mz;
	return true;
}

bool mpu9250_unit::iterate_position(double dt)
{
	if (measure() == false)
		return false;
	imu.iterate(dt, meas);
	return true;
}

void mpu9250_unit::set_errors(float gyro_err, float gyro_drift)
{
	imu.init(gyro_err, gyro_drift);
}
