#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <Eigen/Dense>

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct measurement {
    Vector3f a, m, w;
};

class kalman_filter {
    Vector3f omega;
    Vector3f v;
    Matrix3f I, iI;
    Quaternionf q;
    float mass;
    Vector3f B2;
    float R2; // errors of estimation
    float E2, F2; // error of measure and vibration of orientation
    Vector3f N2, G2; // error of measure and vibration of omega
public:
    void init_parameters(Matrix3f nI, float nm);
    void init_err(float e2, float f2, Vector3f g2, Vector3f n2);
    void init_pos(Vector3f om, Quaternionf rot);
    void iterate(float dt, Vector3f M, Vector3f F, measurement meas);
    Quaternionf orientation() {return q;}
    Vector3f rotation() {return omega;}
};

Quaternionf find_orientation(float ax, float ay, float az,
			     float mx, float my, float mz);

void print_state(const char *str, const Vector3f w, const Quaternionf q);

#endif
