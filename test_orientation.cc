#include <integrate.h>
#include <iostream>
#include <stdio.h>

int main(void)
{
	orientation_unit orient;

	Vector3d w;
	Vector3d bottom, north;
	double tmax = 10, t, dt = 0.1;

	w(0) = w(1) = 0;
	w(2) = 1;

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
		orient.kalman_step(w, Pw, bottom, north, Pb, Pn, dt);
		Quaterniond rot = orient.get_orientation();
		Matrix3d R = rot.toRotationMatrix();
		/*printf( "%5.3lf %5.3lf %5.3lf\n"
			"%5.3lf %5.3lf %5.3lf\n"
			"%5.3lf %5.3lf %5.3lf\n\n",
			R(0,0), R(0,1), R(0,2),
			R(1,0), R(1,1), R(1,2),
			R(2,0), R(2,1), R(2,2));*/
		printf("%lf %lf\n", t, atan2(-R(0,1), R(0,0)));
	}

	return 0;
}
