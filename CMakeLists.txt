cmake_minimum_required(VERSION 2.6)
project(mpu_9250)

set(TOOCHAIN_BIN /home/vlad/module/toolchain/arm-module-linux-gnueabi/bin)
set(CROSS_PREFIX ${TOOCHAIN_BIN}/arm-module-linux-gnueabi-)
set(CMAKE_C_COMPILER ${CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_PREFIX}g++)

include_directories(${PROJECT_SOURCE_DIR})
add_executable(imu_measure main.c mpu_9250.c)
add_executable(set_bp set_bp.c)

install(TARGETS imu_measure set_bp RUNTIME DESTINATION bin)
