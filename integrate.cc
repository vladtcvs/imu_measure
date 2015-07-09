#include <integrate.h>
#include <mpu_9250.h>

imu_unit::imu_unit(std::string devname, Vector3d nI, Vector3d nw, Matrix3d nP,
		   Matrix3d nR, Vector3d nv)
	: gyro(nw, nP, nI),
	  R(nR), v(nv)
{
	fd = open_imu(devname.c_str());
	if (fd == 0)
		throw 1;
	enable_compass(fd, 1);
	setup_imu(fd, 0, 1);
	offset.ax = 0;
	offset.ay = 0;
	offset.az = 0;

	offset.mx = 0;
	offset.my = 0;
	offset.mz = 0;

	offset.wx = 0;
	offset.wy = 0;
	offset.wz = 0;
}

imu_unit::~imu_unit()
{
	if (fd)
		close_imu(fd);
}

imu_unit::imu_unit()
{
	int i, j;
	for (i = 0; i < 3; i++)
	for (j = 0; j < 3; j++)
		R(i, j) = (i == j);
	v.setZero();
	a.setZero();
	a(2) = 10;
	fd = open_imu("/dev/i2c-2");
	if (fd == 0)
		throw 1;
	offset.ax = 0;
	offset.ay = 0;
	offset.az = 0;

	offset.mx = 0;
	offset.my = 0;
	offset.mz = 0;

	offset.wx = 0;
	offset.wy = 0;
	offset.wz = 0;
}

void imu_unit::to_local(orient_data_t* data)
{
	double tx = (data->wx + data->wy)/1.414;
	double ty = (data->wx - data->wy)/1.414;
	double tz = data->wz;

	data->wx = tx;
	data->wy = ty;
	data->wz = tz;

	tx = (data->ax + data->ay)/1.414;
	ty = (data->ax - data->ay)/1.414;
	tz = data->az;

	data->ax = tx;
	data->ay = ty;
	data->az = tz;

	tx = (data->mx + data->my)/1.414;
	ty = (data->mx - data->my)/1.414;
	tz = data->mz;

	data->mx = tx;
	data->my = ty;
	data->mz = tz;
}


bool imu_unit::measure_offset(const int N)
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
	to_local(&offset);
	return true;
}


bool imu_unit::measure()
{
	orient_data_t dc;
	if (read_imu(fd, &dc) < 0)
		return false;
	to_local(&dc);
	meas.ax = dc.ax - offset.ax;
	meas.ay = dc.ay - offset.ay;
	meas.az = dc.az - offset.az;

	meas.wx = dc.wx - offset.wx;
	meas.wy = dc.wy - offset.wy;
	meas.wz = dc.wz - offset.wz;

	meas.mx = dc.mx - offset.mx;
	meas.my = dc.my - offset.my;
	meas.mz = dc.mz - offset.mz;
	return true;
}

void imu_unit::get_position()
{
	measure();
}
