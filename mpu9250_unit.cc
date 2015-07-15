#include <mpu9250_unit.h>
#include <iostream>

mpu9250_unit::mpu9250_unit(std::string devname, double mass, Vector3d I, Vector3d w, Matrix3d R, Vector3d v)
{
	imu = new imu_unit();
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
		delete imu;
		throw 1;
	}
	enable_compass(fd, 1);
	setup_imu(fd, GFS_250_DPS, AFS_2G);
}

mpu9250_unit::mpu9250_unit()
{
	imu = new imu_unit;
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
		delete imu;
		throw 1;
	}
	enable_compass(fd, 1);
	setup_imu(fd, GFS_250_DPS, AFS_2G);
}

mpu9250_unit::~mpu9250_unit()
{
	if (imu)
		delete imu;
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
	to_local(&offset);
	return true;
}


bool mpu9250_unit::measure()
{
	orient_data_t dc;
	if (read_imu(fd, &dc) < 0)
		return false;
	to_local(&dc);
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

void mpu9250_unit::to_local(orient_data_t* data)
{
/*	double tx = (data->wx + data->wy)/1.414;
	double ty = (data->wx - data->wy)/1.414;
	double tz = data->wz;

	data->wx = tx;
	data->wy = ty;
	data->wz = tz;

	tx = (data->ax + data->ay)/1.414;
	ty = (data->ax - data->ay)/1.414;
	tz = data->az;

	data->ax = tx;
	data->ay = -ty;
	data->az = -tz;

	tx = (data->mx + data->my)/1.414;
	ty = (data->mx - data->my)/1.414;
	tz = data->mz;

	data->mx = tx;
	data->my = ty;
	data->mz = tz;*/
}

bool mpu9250_unit::get_position(const Vector3d &M, const Vector3d &F, double dt,
			    double noise_w, double noise_a, double noise_m,
			    double vibrating_w, double vibrating_a, double vibrating_m)
{
	if (measure() == false)
		return false;
	Vector3d w_meas(meas.wx, meas.wy, meas.wz);
	Matrix3d Rw, Qw;
	Matrix3d Ra, Qa;
	Matrix3d Rm, Qm;
	SetIdentityError(Rw, vibrating_w);
	SetIdentityError(Ra, vibrating_a);
	SetIdentityError(Rm, vibrating_m);

	SetIdentityError(Qw, noise_w);
	SetIdentityError(Qa, noise_a);
	SetIdentityError(Qm, noise_m);

	Vector3d m_meas(meas.mx, meas.my, meas.mz);
	Vector3d a_meas(meas.ax, meas.ay, meas.az);

	// TODO: Here we need substract centrifugal force
	Vector3d bottom = a_meas / a_meas.norm();
	Vector3d north = m_meas / m_meas.norm();
	Matrix3d Pb, Pn;
	std::cout<<"a: "<<a_meas(0)<<" "<<a_meas(1)<<" "<<a_meas(2)<<"\n";
//	std::cout<<"north: "<<north(0)<<" "<<north(1)<<" "<<north(2)<<"\n";

	imu->step(M, F, dt, w_meas, a_meas, m_meas, Rw, Ra, Rm, Qw, Qa, Qm);

	return true;
}
