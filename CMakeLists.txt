cmake_minimum_required(VERSION 2.6)
project(mpu_9250)

include_directories(${PROJECT_SOURCE_DIR})
add_executable(imu_measure main.c mpu_9250.c)
add_executable(set_bp set_bp.c)

install(TARGETS imu_measure set_bp RUNTIME DESTINATION bin)
