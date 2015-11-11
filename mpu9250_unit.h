#ifndef __MPU9250_UNIT__
#define __MPU9250_UNIT__

#include <integrate.h>

class mpu9250_unit {
	int fd;
	float angle, cosa, sina;
	madgwick imu;
	orient_data_t offset;
	float adjmx, adjmy, adjmz;
	orient_data_t to_local(orient_data_t tmp);
public:
	orient_data_t meas;
	bool measure();

	mpu9250_unit();
	mpu9250_unit(std::string devname);
	~mpu9250_unit();
	bool measure_offset(const int N);
	void set_errors(float gyro_err, float gyro_drift,
			float amx, float amy, float amz);
	bool iterate_position(double dt);
	const Quaterniond orientation() {return imu.orientation();}
};

#endif
