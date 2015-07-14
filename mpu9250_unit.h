#ifndef __MPU9250_UNIT__
#define __MPU9250_UNIT__

#include <integrate.h>

class mpu9250_unit {
	int fd;
	imu_unit *imu;
	orient_data_t offset;
	orient_data_t meas;
	void to_local(orient_data_t *data);
public:
	mpu9250_unit();
	mpu9250_unit(std::string devname, double mass, Vector3d I, Vector3d w,
		 Matrix3d R, Vector3d v);
	~mpu9250_unit();
	bool measure_offset(const int N);
	bool measure();
	bool get_position(const Vector3d &M, const Vector3d &F, double dt,
			  double noise_w, double noise_a, double noise_m,
			  double vibrating_w, double vibrating_a, double vibrating_m);
	const Quaterniond& orientation() {return imu->orientation();};
};

#endif
