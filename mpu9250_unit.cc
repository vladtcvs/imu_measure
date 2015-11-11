#include <mpu9250_unit.h>
#include <iostream>

mpu9250_unit::mpu9250_unit(std::string devname)
{
	adjmx = 0;
	adjmy = 0;
	adjmz = 0;

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
	cosa = 1;
	sina = 0;
}

mpu9250_unit::mpu9250_unit()
{
	adjmx = 0;
	adjmy = 0;
	adjmz = 0;

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
	cosa = 1;
	sina = 0;
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

	offset.mx -= adjmx;
	offset.my -= adjmy;
	offset.mz -= adjmz;

	angle = imu.set_start(offset.ax, offset.ay, offset.az,
		      offset.mx, offset.my, offset.mz);
	cosa = cos(angle);
	sina = sin(angle);
	return true;
}

orient_data_t mpu9250_unit::to_local(orient_data_t tmp)
{
	orient_data_t res;
	res.az = tmp.az;
	res.mz = tmp.mz;
	res.wz = tmp.wz;

	res.ax = tmp.ax * cosa + tmp.ay * sina;
	res.ay = tmp.ay * cosa - tmp.ax * sina;
	
	res.mx = tmp.mx * cosa + tmp.my * sina;
	res.my = tmp.my * cosa - tmp.mx * sina;
	
	res.wx = tmp.wx * cosa + tmp.wy * sina;
	res.wy = tmp.wy * cosa - tmp.wx * sina;
	return res;
}

bool mpu9250_unit::measure()
{
	orient_data_t tmp;
	orient_data_t dc;
	if (read_imu(fd, &dc) < 0)
		return false;

	tmp.ax = dc.ax;
	tmp.ay = dc.ay;
	tmp.az = dc.az;

	tmp.wx = dc.wx - offset.wx;
	tmp.wy = dc.wy - offset.wy;
	tmp.wz = dc.wz - offset.wz;

	tmp.mx = dc.mx - adjmx;
	tmp.my = dc.my - adjmy;
	tmp.mz = dc.mz - adjmz;

	meas = to_local(tmp);
	//meas = tmp;
	return true;
}

bool mpu9250_unit::iterate_position(double dt)
{
	if (measure() == false)
		return false;
	imu.iterate(dt, meas);
	return true;
}

void mpu9250_unit::set_errors(float gyro_err, float gyro_drift,
			      float amx, float amy, float amz)
{
	adjmx = amx;
	adjmy = amy;
	adjmz = amz;
	imu.init(gyro_err, gyro_drift);
}
