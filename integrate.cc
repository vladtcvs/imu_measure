#include <integrate.h>
#include <mpu_9250.h>
#include <math.h>
#include <iostream>

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
	//std::cout<<"rot: "<<rot.w()<<" ("<<rot.x()<<", "<<rot.y()<<", "<<rot.z()<<")\n";
	Quaterniond wgq = rot * wq * (rot.inverse());
	Vector3d wg = wgq.vec();
	//std::cout<<"wgq: "<<wgq.w()<<" ("<<wgq.x()<<", "<<wgq.y()<<", "<<wgq.z()<<")\n";
	assert(fabs(wgq.w()) < 1e-12);

	/* Finding direction and angle of rotation */
	Quaterniond rot_c;
	double wglen = wg.norm();
	if (fabs(wglen) > 1e-6) {
		double angle = wglen * dt;
		Vector3d axis = wg / wglen;
		double sina = sin(angle/2);
		double cosa = cos(angle/2);

		/* Finding calculated new orientation */
		Quaterniond W(cosa, sina*axis(0), sina*axis(1), sina*axis(2));
		rot_c = W * rot; // calculated orientation
	} else {
		rot_c = rot;
	}

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

	// temporary solution
	rot = rot_c;
	//std::cout<<"\n------\n\n";
}

const Quaterniond& orientation_unit::get_orientation()
{
	return rot;
}


imu_unit::imu_unit(double nmass, Vector3d& nI, Vector3d& nw,
		   Quaterniond& nR, Vector3d& nv)
{
	Matrix3d nP;
	nP.setZero();
	gyro = new gyro_unit(nw, nP, nI);
	if (gyro == NULL)
		throw 2;
	orient = new orientation_unit(nR);
	if (orient == NULL) {
		delete gyro;
		throw 2;
	}
	mass = nmass;
}

imu_unit::~imu_unit()
{
	if (gyro)
		delete gyro;
}

imu_unit::imu_unit()
{
	Matrix3d nP;
	Vector3d nw, nI;
	nP.setZero();
	nw.setZero();
	nI(0) = nI(1) = nI(2) = 1;
	Quaterniond nR;
	nR.setIdentity();

	gyro = new gyro_unit(nw, nP, nI);
	if (gyro == NULL)
		throw 2;
	orient = new orientation_unit(nR);
	if (orient == NULL) {
		delete gyro;
		throw 2;
	}
	mass = 1;
}

void imu_unit::step(const Vector3d& M, const Vector3d& F, double dt,
		    const Vector3d& w, const Vector3d& a, const Vector3d& m,
		    const Matrix3d& Rw, const Matrix3d& Ra, const Matrix3d& Rm,
		    const Matrix3d& Qw, const Matrix3d& Qa, const Matrix3d& Qm)
{
	Matrix3d Qb = Qa / a.norm();
	Vector3d bottom = a / a.norm();

	Matrix3d Qn = Qm / m.norm();
	Vector3d north = m / m.norm();
//	orient->kalman_step(gyro->w, gyro->P, bottom, north, Qb, Qn, dt);
//	gyro->kalman_step(w, Qw, Rw, M, dt);
}

const Vector3d QuaternionToRPY(const Quaterniond& q)
{
	double x, y, z, w;
	x = q.x();
	y = q.y();
	z = q.z();
	w = q.w();
	double roll  = atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
	double pitch = atan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z);
	double yaw   =  asin(2*x*y + 2*z*w);
	
	return Vector3d(roll, pitch, yaw);
}
