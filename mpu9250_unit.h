#ifndef __MPU9250_UNIT__
#define __MPU9250_UNIT__

#include <integrate.h>

class mpu9250_unit {
	int fd;
	madgwick imu;
	orient_data_t offset;
	orient_data_t meas;
	bool measure();
public:
	mpu9250_unit();
	mpu9250_unit(std::string devname);
	~mpu9250_unit();
	bool measure_offset(const int N);
	void set_errors(float gyro_err, float gyro_drift);
	bool iterate_position(double dt);
	const Quaterniond orientation() {return imu.orientation();}
};

#endif
