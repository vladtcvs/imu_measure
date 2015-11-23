#ifndef __MPU9250_UNIT__
#define __MPU9250_UNIT__

#include <mpu_9250.h>
#include <kalman.h>

class mpu9250_unit {
    int fd;
    kalman_filter imu;
    float adjmx, adjmy, adjmz;
    float adjax, adjay, adjaz;
    orient_data_t meas;
    bool measure();
public:
    orient_data_t offset;
    Quaternionf initial;
    mpu9250_unit();
    mpu9250_unit(std::string devname);
    ~mpu9250_unit();
    bool measure_offset(const int N);
    /*void set_errors(float gyro_err, float gyro_drift,
		    float aax, float aay, float aaz,
		    float amx, float amy, float amz);*/
    void set_errors(float E2, float F2, Vector3f G2, Vector3f N2,
		    float aax, float aay, float aaz,
		    float amx, float amy, float amz);
    void set_parameters(Matrix3f I, float mass);
    bool iterate_position(double dt, Vector3f M, Vector3f F);
    const Quaternionf orientation() {return imu.orientation();}
};

#endif
