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
#include <cfg.h>

static struct timeval tv1,tv2,dtv;
static struct timezone tz;
stabilizer *stab;


Vector3f QuaternionToRPY(const Quaternionf& q)
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

static void signal_hdl(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		if (stab) {
			stab->turn_off();
			delete stab;
		}
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
	control_config cfg;

	if (read_parse_config("control.cfg", &cfg) == false) {
		return 0;
	}
	stab = new stabilizer("/dev/spidev0.0", 10000, 22);
	stab->set_parameters(1, 0, 0, 0);
	stab->set_hardness(cfg.rigidity_roll, cfg.rigidity_pitch, cfg.rigidity_yaw);

	mpu9250_unit imu("/dev/i2c-2");
	imu.set_errors(cfg.E2, cfg.F2, Vector3f(cfg.G2x, cfg.G2y, cfg.G2z),
			Vector3f(cfg.N2x, cfg.N2y, cfg.N2z),
			cfg.adjax, cfg.adjay, cfg.adjaz,
			cfg.adjmx, cfg.adjmy, cfg.adjmz);

	imu.measure_offset(cfg.measures_start);

	Vector3f rpy = QuaternionToRPY(imu.orientation());
	printf("RPY = %lf %lf %lf\n",
	       rpy(0)*180/M_PI, rpy(1)*180/M_PI, rpy(2)*180/M_PI);

	time_start();
	tp = time_stop()/1000;
	while (1) {
		orient_data_t orient;
		Vector3f M;
		Vector3f force;
		M.setZero();
		force.setZero();
		t = time_stop()/1000.0;
		double dt = t - tp;
		if (imu.iterate_position(dt, M, force)) {
			const Quaternionf rot = imu.orientation();
			Vector3f rpy = QuaternionToRPY(rot);
			stab->current_rpy(rpy(0), rpy(1), rpy(2));
			printf("RPY: %f %f %f\n", rpy(0)*180/M_PI, rpy(1)*180/M_PI, rpy(2)*180/M_PI);
		}
		tp = t;
	}
	
	return 0;
}
