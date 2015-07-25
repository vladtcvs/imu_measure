#include <integrate.h>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

double drand()
{
	double x = rand();
	return (x / RAND_MAX)*2 - 1;
}

int main(void)
{
	Quaterniond rot;
	rot.setIdentity();
	orientation_unit orient(rot);

	Vector3d w;
	Vector3d bottom, north;
	double tmax = 10, t, dt = 0.1;
	double da = 0.01;
	double dz = 0.01;

	w(0) = 1;
	w(1) = -1;
	w(2) = 0;

	Matrix3d Pw;
	SetIdentityError(Pw, 0.1);

	Matrix3d Pb;
	SetIdentityError(Pb, 0.1);
	Matrix3d Pn;
	SetIdentityError(Pn, 0.7);

	bottom(0) = bottom(1) = 0;
	bottom(2) = -1;

	north(0) = 1;
	north(1) = north(2) = 0;

	for (t = 0; t < tmax; t += dt) {
		Quaterniond W;
		double wglen = w.norm();
		if (fabs(wglen) > 1e-6) {
			double angle = wglen * dt;
			Vector3d axis = w / wglen;
			double sina = sin(angle/2);
			double cosa = cos(angle/2);

			/* Finding calculated new orientation */
			W = Quaterniond(cosa, sina*axis(0), sina*axis(1), sina*axis(2));
		} else {
			W.setIdentity();
		}

		double ang = drand()*da*dt;
		Quaterniond vibrating(cos(ang/2),
				      sin(ang/2)*drand(),
				      sin(ang/2)*drand(),
				      sin(ang/2)*drand());
		rot = vibrating*W*rot;


		Quaterniond I(0,1,0,0), J(0,0,1,0), K(0,0,0,1);
		Vector2d sinx;
		Matrix2d Psinx;
		sinx(0) = -(rot*I*rot.inverse()).z();
		sinx(1) = -(rot*J*rot.inverse()).z();
		Psinx.setZero();
		Psinx(0, 0) = dz*dz;
		Psinx(1, 1) = dz*dz;


		orient.kalman_step_global(w, Pw, sinx, Psinx, dt);
		Quaterniond crot = orient.get_orientation();
		Vector3d rpy = QuaternionToRPY(crot);
		printf("%lf %lf %lf %lf\n", t, rpy(0), rpy(1), rpy(2));
		//printf("%lf %lf %lf %lf %lf\n", t, crot.w(), crot.x(), crot.y(), crot.z());
	}

	return 0;
}
