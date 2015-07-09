#ifndef __FILTER_H__
#define __FILTER_H__

#include <Eigen/Dense>

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class gyro_unit {
	Vector3d I;
public:
	Vector3d w;
	Matrix3d P;

	gyro_unit();
	gyro_unit(Vector3d nw, Matrix3d nP, Vector3d nI);
	void kalman_step(Vector3d z, Matrix3d Q,
			 Matrix3d R, Vector3d M, double dt);
};

extern const double eps[3][3][3];

#endif