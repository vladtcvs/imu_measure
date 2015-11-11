#include <stdio.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <mpu_9250.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <set_pwm.h>
#include <mpu9250_unit.h>
#include <stabilization.h>

static struct timeval tv1,tv2,dtv;
static struct timezone tz;

static int fd;

const Vector3d QuaternionToRPY(const Quaterniond& q)
{
	double q0, q1, q2, q3;
	q1 = q.x();
	q2 = q.y();
	q3 = q.z();
	q0 = q.w();
	double roll  = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*q1*q1 - 2*q2*q2);
	double pitch = asin(2*q0*q2 - 2*q1*q3);
	double yaw   =  atan2(2*q0*q3 + 2*q1*q2, 1 - 2*q2*q2 - 2*q3*q3);
	
	return Vector3d(roll, pitch, yaw);
}

static void signal_hdl(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
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
	double t, tp;
	signal(SIGINT, signal_hdl);

	//stabilizer stab("/dev/spi0.0");
	mpu9250_unit imu("/dev/i2c-2");
	imu.set_errors(5, 0.2, -89, 54, 254);
	imu.measure_offset(20);
	/*fd = open_imu("/dev/i2c-2");
	setup_imu(fd, GFS_250_DPS, AFS_2G);
	enable_compass(fd, 1);*/
	time_start();
	tp = time_stop()/1000;
	while (1) {
		orient_data_t orient;
		t = time_stop()/1000.0;
		double dt = t - tp;
		//imu.measure();
		//printf("%lf %lf %lf %lf\n", t, imu.meas.mx, imu.meas.my, imu.meas.mz);
		if (imu.iterate_position(dt)) {
			const Quaterniond rot = imu.orientation();
			Vector3d rpy = QuaternionToRPY(rot);
			printf("%lf %lf %lf %lf\n", t, rpy(0)*180/3.141, rpy(1)*180/3.141, rpy(2)*180/3.141);
		}
/*
		if (read_imu(fd, &orient) < 0) {
			printf("Error reading\n");
			break;
		}
		printf("%lf %i %i %i %i %i %i\n", t,
			orient.mx, orient.my, orient.mz,
			orient.ax, orient.ay, orient.az);*/
		tp = t;
	}
	//close_imu(fd);
	return 0;
}
