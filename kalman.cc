#include <kalman.h>
#include <stdio.h>

#define SQR(x) ((x)*(x))

static Vector3f QuaternionToRPY(const Quaternionf& q)
{
	float q0, q1, q2, q3;
	q1 = q.x();
	q2 = q.y();
	q3 = q.z();
	q0 = q.w();
	float roll  = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*q1*q1 - 2*q2*q2);
	float pitch = asin(2*q0*q2 - 2*q1*q3);
	float yaw   =  atan2(2*q0*q3 + 2*q1*q2, 1 - 2*q2*q2 - 2*q3*q3);
	
	return Vector3f(roll, pitch, yaw);
}

void print_state(const char *str, const Vector3f w, const Quaternionf q)
{
	Vector3f rpy = QuaternionToRPY(q);
	printf("%s: RPY = [%f %f %f] w = [%f %f %f]\n", str,
		rpy(0)*180/M_PI, rpy(1)*180/M_PI, rpy(2)*180/M_PI,
		w(0) * 180/M_PI, w(1) * 180/M_PI, w(2) * 180/M_PI);
}

Quaternionf find_orientation(float ax, float ay, float az,
			     float mx, float my, float mz)
{
	float gamma, cosg, angle;
	float al;
	Quaternionf n;

	al = sqrt(ax*ax + ay*ay + az*az);
	ax /= al;
	ay /= al;
	az /= al;

	cosg = -az;
	gamma = acos(cosg);

	if (gamma < 1e-6) {
		n.w() = 1;
		n.x() = n.y() = n.z() = 0;
	} else {
		float nl = sqrt(ax*ax + ay*ay);
		n.w() = cos(gamma/2);
		n.x() = sin(gamma/2) * ay / nl;
		n.y() = -sin(gamma/2) * ax / nl;
		n.z() = 0;
	}

	Quaternionf m(0, mx, my, mz), mr;	
	mr = n*m*n.inverse();

	angle = atan2(mr.y(), mr.x());
	Quaternionf rot(cos(angle/2), 0, 0, -sin(angle/2));
	return rot*n;
}

void kalman_filter::init_parameters(Matrix3f nI, float nm)
{
    I = nI;
    iI = I.inverse();
    mass = nm;
}

void kalman_filter::init_err(float e2, float f2, Vector3f g2, Vector3f n2)
{
    E2 = e2;
    F2 = f2;
    G2 = g2;
    N2 = n2;
}

void kalman_filter::init_pos(Vector3f om, Quaternionf rot)
{
    omega = om;
    q = rot;
    R2 = 0;
    B2(0) = 0;
    B2(1) = 0;
    B2(2) = 0;
}

const int eps[3][3][3] = {
    {{0, 0, 0},
    {0, 0, 1},
    {0, -1, 0}},

    {{0, 0, -1},
    {0, 0, 0},
    {1, 0, 0}},

    {{0, 1, 0},
    {-1, 0, 0},
    {0, 0, 0}}
};

Quaternionf operator + (Quaternionf a, Quaternionf b)
{
    return Quaternionf(a.w() + b.w(), a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
}

Quaternionf operator * (Quaternionf a, float b)
{
    return Quaternionf(a.w()*b, a.x()*b, a.y()*b, a.z()*b);
}

Quaternionf operator / (Quaternionf a, float b)
{
     return Quaternionf(a.w()/b, a.x()/b, a.y()/b, a.z()/b);
}

static float scalar(Quaternionf a, Quaternionf b)
{
    return a.w()*b.w() + a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
}

static Vector3f rotate(Vector3f v, Quaternionf q)
{
    Quaternionf v_q(0, v(0), v(1), v(2));
    Quaternionf vqe = q.inverse()*v_q*q;
    return Vector3f(vqe.x(), vqe.y(), vqe.z());
}

void kalman_filter::iterate(float dt, Vector3f M, Vector3f F, measurement meas)
{
    Vector3f omega_est;
    Quaternionf q_est, Om, new_q;
    float w, sinw;

    printf("************************\n");
    /* Extrapolation stage */
    Vector3f omega_dt = iI * M;
    for (int i = 0; i < 3; i++) {
	for (int s = 0; s < 3; s++)
	for (int t = 0; t < 3; t++)
	for (int p = 0; p < 3; p++)
	for (int m = 0; m < 3; m++) {
	    omega_dt(i) -= iI(i, s) * eps[s][t][p] * I(p, m) * omega(t) * omega(m);
	}
    }
    omega_est = omega + omega_dt * dt;
    w = omega.norm();
    if (w > 1e-12) {
	Vector3f omega_global = rotate(omega, q);
	Om.w() = cos(w * dt/2);
	sinw = sin(w * dt/2);
	Om.x() = sinw * omega_global(0) / w;
	Om.y() = sinw * omega_global(1) / w;
	Om.z() = sinw * omega_global(2) / w;

	q_est = Om * q;
    } else {
	q_est = q;
    }
    print_state("Est ", omega_est, q_est);

    /* Using measurements stage */
    Quaternionf q_meas = find_orientation(meas.a(0), meas.a(1), meas.a(2),
					  meas.m(0), meas.m(1), meas.m(2));
    Vector3f omega_meas = meas.w;

    print_state("Meas", omega_meas, q_meas);
    /* calculating alpha - coefficient for omega */
    float B = 0, D;
    Vector3f Bq;
    float alpha;
    for (int g = 0; g < 3; g++) {
	Bq(g) = 0;
	for (int m = 0; m < 3; m++) {
	    D = (g == m);
	    for (int s = 0; s < 3; s++)
	    for (int p = 0; p < 3; p++)
	    for (int t = 0; t < 3; t++)
		D -= dt * iI(g,s)*(eps[s][m][p]*I(p,t) + eps[s][t][p]*I(p,m)) * omega(t);
	    B += D*D * B2(m);
	    Bq(g) += D*D * B2(m);
	}
    }
    alpha = B/(B + N2(0) + N2(1) + N2(2) + G2(0) + G2(1) + G2(2));

    /* calculating tau - coefficient for q*/
    float c = scalar(q_est, q_meas);
    float phi = acos(c);
    float a = 1 - F2/2 - R2/2 - (B2(0) + B2(1) + B2(2))*dt*dt/8;
    float b = 1 - E2/2;
    float x = a * sqrt(1-c*c)/sqrt(a*a + b*b - 2*a*b*c);
    printf("x = %f c = %f a = %f b = %f\n", x, c, a, b);
    float tau;
    if ((1 - c) < 1e-12)
	tau = 0;
    else
	tau = acos(x)/phi;

    printf("tau = %f alpha = %f\n", tau, alpha);
    if (alpha < 0)
	    alpha = 0;
    if (alpha > 1)
	    alpha = 1;

    if (tau < 0)
	    tau = 0;
    if (tau > 1)
	    tau = 1;
    /* Finding new estimated values */
    Vector3f new_omega = omega_est * (1 - alpha) + omega_meas * alpha;

    float k1 = sin((1-tau)*phi)/sin(phi);
    float k2 = sin(tau*phi)/sin(phi);

    if (fabs(phi) > 1e-6) {
	new_q = q_est * k1 + q_meas * k2;
    } else {
	new_q.w() = 1;
	new_q.x() = 0;
	new_q.y() = 0;
	new_q.z() = 0;
    }
    print_state("New ", new_omega, new_q);
    /* Finding new errors */
    R2 = k1*a + k2*b - 1;
    if (R2 < 0)
	R2 = 0;
    for (int i = 0; i < 3; i++) {
	B2(i) = SQR(1-alpha) * Bq(i) + SQR(alpha) * (G2(i) + N2(i));
    }
}
