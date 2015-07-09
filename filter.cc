#include <filter.h>

const double eps[3][3][3] = {	{{0, 0,  0}, { 0, 0, 1}, {0, -1, 0}},
				{{0, 0, -1}, { 0, 0, 0}, {1,  0, 0}},
				{{0, 1,  0}, {-1, 0, 0}, {0,  0, 0}}
			};

#define SQR(x) ((x)*(x))

gyro_unit::gyro_unit()
{
	w.setZero();
	P.setZero();
	I.setOnes();
}


gyro_unit::gyro_unit(Vector3d nw, Matrix3d nP, Vector3d nI) :
	w(nw),
	P(nP),
	I(nI)
{
}

void gyro_unit::kalman_step(Vector3d z, Matrix3d Q,
			    Matrix3d R, Vector3d M, double dt)
{
	Matrix3d Lambda;
	int q, j, m, n, i, k;
	Vector3d f(M(0)/I(0)*dt, M(1)/I(1)*dt, M(2)/I(2)*dt);

	Lambda.setZero();
	for (q = 0; q < 3; q++)
	for (j = 0; j < 3; j++) {
		double l = 0;
		for (m = 0; m < 3; m++)
		for (n = 0; n < 3; n++) {
			l -= 2 * w(n) * P(j,m) * eps[q][m][n] *
						(I(n) - I(m))/(2*I(q)) * dt;
			l -= 2 * w(n) * P(q,m) * eps[j][m][n] *
						(I(n) - I(m))/(2*I(j)) * dt;
		}
		Lambda(q,j) = l;
	}
	Lambda += P + R;
	Matrix3d LLt = (Lambda + Lambda.transpose());
	Matrix3d QL = 2*Q + LLt;
	Matrix3d K = LLt * (QL.inverse());

	Vector3d wc = w + f;
	for (i = 0; i < 3; i++) {
		double dw = 0;
		for (j = 0; j < 3; j++)
		for (k = 0; k < 3; k++)
			dw -= eps[i][j][k]*(I(k)-I(j))/(2*I(i))*dt*w(j)*w(k);
		wc(i) += dw;
	}

	Matrix3d E;
	for (i = 0; i < 3; i++)
	for (j = 0; j < 3; j++)
		E(i,j) = (i == j);

	Vector3d FP;
	FP.setZero();
	for (j = 0; j < 3; j++) {
		for (m = 0; m < 3; m++)
		for (n = 0; n < 3; n++)
			FP(j) -= P(m,n) * eps[j][m][n]*(I(n)-I(m))/(2*I(j))*dt;
	}

	Matrix3d T = (E-K);
	Vector3d ws = T*wc + K*z + T * FP;
	Matrix3d Ps = T * Lambda * (T.transpose()) + K * Q * (K.transpose());
	w = ws;
	P = Ps;
}

void SetIdentityError(Matrix3d& M, double de)
{
	int i, j;
	for (i = 0; i < 3; i++)
	for (j = 0; j < 3; j++)
		M(i,j) = (i == j) * de * de;
}
