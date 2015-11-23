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

	offset.ax -= adjax;
	offset.ay -= adjay;
	offset.az -= adjaz;

	Quaternionf init_q = find_orientation(offset.ax, offset.ay, offset.az,
		      offset.mx, offset.my, offset.mz);
	Vector3f init_om;
	init_om.setZero();

	imu.init_pos(init_om, init_q);
	return true;
}

bool mpu9250_unit::measure()
{
	int i;
	int num = 2;
	orient_data_t dc;

	meas.ax = 0;
	meas.ay = 0;
	meas.az = 0;

	meas.wx = 0;
	meas.wy = 0;
	meas.wz = 0;

	meas.mx = 0;
	meas.my = 0;
	meas.mz = 0;

	for (i = 0; i < num; i++) {
		if (read_imu(fd, &dc) < 0)
			return false;

		meas.ax += dc.ax;
		meas.ay += dc.ay;
		meas.az += dc.az;

		meas.wx += dc.wx;
		meas.wy += dc.wy;
		meas.wz += dc.wz;

		meas.mx += dc.mx;
		meas.my += dc.my;
		meas.mz += dc.mz;
	}
	
	meas.ax /= num;
	meas.ay /= num;
	meas.az /= num;

	meas.mx /= num;
	meas.my /= num;
	meas.mz /= num;

	meas.wx /= num;
	meas.wy /= num;
	meas.wz /= num;

	meas.ax -= adjax;
	meas.ay -= adjay;
	meas.az -= adjaz;

	meas.mx -= adjmx;
	meas.my -= adjmy;
	meas.mz -= adjmz;

	meas.wx -= offset.wx;
	meas.wy -= offset.wy;
	meas.wz -= offset.wz;

	//std::cout << meas.ax << " " << meas.ay << " " << meas.az << "\n";
	return true;
}

bool mpu9250_unit::iterate_position(double dt, Vector3f M, Vector3f F)
{
	if (measure() == false)
		return false;
	measurement meas_kalman;
	meas_kalman.a(0) = meas.ax;
	meas_kalman.a(1) = meas.ay;
	meas_kalman.a(2) = meas.az;
	meas_kalman.m(0) = meas.mx;
	meas_kalman.m(1) = meas.my;
	meas_kalman.m(2) = meas.mz;
	meas_kalman.w(0) = meas.wx;
	meas_kalman.w(1) = meas.wy;
	meas_kalman.w(2) = meas.wz;
	imu.iterate(dt, M, F, meas_kalman);
	return true;
}

void mpu9250_unit::set_errors(float E2, float F2, Vector3f G2, Vector3f N2,
			      float aax, float aay, float aaz,
			      float amx, float amy, float amz)
{
	adjmx = amx;
	adjmy = amy;
	adjmz = amz;
	adjax = aax;
	adjay = aay;
	adjaz = aaz;
	imu.init_err(E2, F2, G2, N2);
}
