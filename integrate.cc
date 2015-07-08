#include <integrate.h>

imu_unit::imu_unit(Vector3d nI, Vector3d nw, Matrix3d nP,
		   Matrix3d nR, Vector3d nv)
	: gyro(nw, nP, nI),
	  R(nR), v(nv)
{
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
}
