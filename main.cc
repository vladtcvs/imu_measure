#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <mpu_9250.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>

#include <integrate.h>

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
	Vector3d v;

	if (argc >= 4) {
		sscanf(argv[2], "%lf", &dw);
		sscanf(argv[3], "%lf", &dz);
	}
	M.setZero();
	I.setOnes();
	P.setZero();
	SetIdentityError(Q, dz);
	w.setZero();
	v.setZero();
	imu_unit imu("/dev/i2c-2", I, w, P, R, v);

	double t, tp;
	if (argc < 2)
		f = stdout;
	else
		f = fopen(argv[1], "wt");
	signal(SIGINT, signal_hdl);
	imu.measure_offset(20);

	time_start();
	tp = time_stop()/1000;
	while (1) {
		t = time_stop()/1000.0;
		double dt = t - tp;
		SetIdentityError(R, dw * dt);

		if (imu.measure()) {
			fprintf(f, "%lf %lf %lf %lf\n", 
			       t, imu.gyro_data().w(0), imu.gyro_data().w(1), imu.gyro_data().w(2));
		}
		tp = t;
		//usleep(10000);
		fflush(f);
	}

	return 0;
}

