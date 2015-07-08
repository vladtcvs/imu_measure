#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <mpu_9250.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>

#include <filter.h>

static struct timeval tv1,tv2,dtv;
static struct timezone tz;

static FILE *f;
static int fd;

static void signal_hdl(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		if (f != stdout)
			fclose(f);
		close_imu(fd);
		exit(0);
	}
}

static void time_start(void)
{
	gettimeofday(&tv1, &tz);
}

static long time_stop(void)
{
	gettimeofday(&tv2, &tz);
	dtv.tv_sec = tv2.tv_sec -tv1.tv_sec;
	dtv.tv_usec = tv2.tv_usec - tv1.tv_usec;
	if (dtv.tv_usec < 0) {
		dtv.tv_sec--;
		dtv.tv_usec += 1000000;
	}
	return dtv.tv_sec*1000 + dtv.tv_usec/1000;
}

void measure_stay(int fd, orient_data_t *data)
{
	int i;
	const int N = 20;
	int n = 0;
	data->wx = 0;
	data->wy = 0;
	data->wz = 0;
	data->ax = 0;
	data->ay = 0;
	data->az = 0;
	data->mx = 0;
	data->my = 0;
	data->mz = 0;
	for (i = 0; i < N; i++) {
		orient_data_t dc;
		if (read_imu(fd, &dc) >= 0) {
			data->wx += dc.wx;
			data->wy += dc.wy;
			data->wz += dc.wz;

			data->ax += dc.ax;
			data->ay += dc.ay;
			data->az += dc.az;

			data->mx += dc.mx;
			data->my += dc.my;
			data->mz += dc.mz;
			n++;
		}
	}
	if (n) {
		data->wx /= n;
		data->wy /= n;
		data->wz /= n;

		data->ax /= n;
		data->ay /= n;
		data->az /= n;

		data->mx /= n;
		data->my /= n;
		data->mz /= n;
	} else {
		fprintf(stderr, "No measures\n");
	}
}

void to_local(orient_data_t *data)
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

int main(int argc, char **argv)
{
	int i, j;
	double dz = 0.05;
	double dw = 0.01;
	Vector3d I;
	Matrix3d Q, R;
	Vector3d w;
	Matrix3d P;
	Vector3d M;

	if (argc >= 4) {
		sscanf(argv[2], "%lf", &dw);
		sscanf(argv[3], "%lf", &dz);
	}
	M.setZero();
	I.setOnes();
	for (i = 0; i < 3; i++)
	for (j = 0; j < 3; j++) {
		Q(i,j) = (i==j)*dz*dz;
		P(i,j) = 0;
	}
	w.setZero();
	gyro_unit gyro(w, P, I);

	orient_data_t data, stay;
	double t, tp;
	fd = open_imu("/dev/i2c-2");
	if (fd == 0)
		return 0;
	
	if (argc < 2)
		f = stdout;
	else
		f = fopen(argv[1], "wt");
	signal(SIGINT, signal_hdl);

	enable_compass(fd, 1);
	setup_imu(fd, 0, 1);
	time_start();
	tp = time_stop()/1000;
	measure_stay(fd, &stay);
	to_local(&stay);
	fprintf(stderr, "stay measured\n");
	fprintf(stderr, "%lf %lf %lf\n", stay.wx, stay.wy, stay.wz);
	while (1) {
		t = time_stop()/1000.0;
		double dt = t - tp;
		for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			R(i, j) = (i==j)*dw*dw*dt*dt;
		}

		if (read_imu(fd, &data) >= 0) {
			to_local(&data);
			Vector3d z(data.wx - stay.wx, data.wy - stay.wy,
					data.wz - stay.wz);
			gyro.kalman_step(z, Q, R, M, dt);
			fprintf(f, "%lf %lf %lf %lf %lf %lf %lf\n", 
			       t, z(0), z(1), z(2), gyro.w(0), gyro.w(1), gyro.w(2));
		}
		tp = t;
		//usleep(10000);
		fflush(f);
	}

	return 0;
}

