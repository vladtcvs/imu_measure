#include <integrate.h>
#include <mpu_9250.h>
#include <math.h>

orientation_unit::orientation_unit(const Quaterniond& nrot)
{
	Prot.setZero();
	rot = nrot;
}

orientation_unit::orientation_unit()
{
	Prot.setZero();
	rot.setIdentity();
}

void orientation_unit::kalman_step(const Vector3d& w, const Matrix3d& Pw,
				   const Vector3d& bottom, const Vector3d& north,
				   const Matrix3d& Pb, const Matrix3d& Pn,
				   double dt)
{
	/* Getting w in global coordinate system */
	Quaterniond wq(0, w(0), w(1), w(2));
	Quaterniond wgq = rot * wq * (rot.inverse());
	Vector3d wg = wgq.vec();
	assert(fabs(wgq.w()) < 1e-12);

	/* Finding direction and angle of rotation */
	double wglen = wg.norm();
	double angle = wglen * dt;
	Vector3d axis = wg / wglen;
	double sina = sin(angle/2);
	double cosa = cos(angle/2);

	/* Finding calculated new orientation */
	Quaterniond W(cosa, sina*axis(0), sina*axis(1), sina*axis(2));
	Quaterniond rot_c = W * rot; // calculated orientation

	/* Finding cos(a) and cos(b) for calculated orientation */
	Quaterniond i(0, 1, 0, 0), j(0, 0, 1, 0), k(0, 0, 0, 1);
	double cosa_c = -(rot_c * i * (rot_c.inverse())).z();
	double cosb_c = -(rot_c * j * (rot_c.inverse())).z();

	/* Finding measured cos(a) and cos(b) */
	double bottom_len = bottom.norm();
	double cosa_z = bottom(0) / bottom_len;
	double cosb_z = bottom(1) / bottom_len;

	/* difference vector */
	Vector2d dcos(cosa_z - cosa_z, cosb_z - cosb_c);
}

const Quaterniond& orientation_unit::get_orientation()
{
	return rot;
}


imu_unit::imu_unit(std::string devname, double mass, Vector3d nI, Vector3d nw,
		   Matrix3d nR, Vector3d nv)
{
	Matrix3d nP;
	nP.setZero();
	fd = open_imu(devname.c_str());
	if (fd == 0)
		throw 1;
	gyro = new gyro_unit(nw, nP, nI);
	if (gyro == NULL)
		throw 2;
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

	m = mass;
}

imu_unit::~imu_unit()
{
	if (fd)
		close_imu(fd);
	if (gyro)
		delete gyro;
}

imu_unit::imu_unit()
{
	int i, j;
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

bool imu_unit::get_position(const Vector3d &M, const Vector3d &F, double dt,
			    double noise_w, double noise_a, double noise_m,
			    double vibrating_w, double vibrating_a, double vibrating_m)
{
	if (measure() == false)
		return false;
	Vector3d w_meas(meas.wx, meas.wy, meas.wz);
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

	orient->kalman_step(gyro->w, gyro->P, bottom, north, Pb, Pn, dt);
	gyro->kalman_step(w_meas, Qw, Rw, M, dt);
	return true;
}

const Vector3d imu_unit::orientation()
{
	Vector3d rpy;
	rpy.setZero();
	return rpy;
}
