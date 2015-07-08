cmake_minimum_required(VERSION 2.6)
project(mpu_9250)

set(CMAKE_SYSTEM_INCLUDE_PATH /home/vlad/module/buildroot/buildroot/output/host/usr/arm-buildroot-linux-gnueabi/sysroot/usr/include)
set(TOOCHAIN_BIN /home/vlad/module/buildroot/buildroot/output/host/usr/bin)
set(CROSS_PREFIX ${TOOCHAIN_BIN}/arm-buildroot-linux-gnueabi-)
set(CMAKE_C_COMPILER ${CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_PREFIX}g++)

include_directories(${PROJECT_SOURCE_DIR} ${CMAKE_SYSTEM_INCLUDE_PATH}/eigen3)

add_library(mpu9250 STATIC mpu_9250.c)

add_executable(imu_measure main.cc integrate.cc)
add_executable(set_bp set_bp.c)

add_dependencies(imu_measure mpu9250)
add_dependencies(set_bp mpu9250)

target_link_libraries(imu_measure mpu9250 ${PROJECT_SOURCE_DIR}/libfilter.a)
target_link_libraries(set_bp mpu9250)

install(TARGETS imu_measure set_bp RUNTIME DESTINATION bin)
install(TARGETS mpu9250 ARCHIVE DESTINATION lib)
