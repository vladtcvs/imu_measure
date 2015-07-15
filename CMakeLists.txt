cmake_minimum_required(VERSION 2.6)
project(mpu_9250)

add_definitions(-DDEBUG_GYRO=0)
#set(CMAKE_SYSTEM_INCLUDE_PATH /usr/include/)
set(CMAKE_SYSTEM_INCLUDE_PATH /home/vlad/module/buildroot/buildroot/output/host/usr/arm-buildroot-linux-gnueabi/sysroot/usr/include)
set(TOOCHAIN_BIN /home/vlad/module/buildroot/buildroot/output/host/usr/bin)
set(CROSS_PREFIX ${TOOCHAIN_BIN}/arm-buildroot-linux-gnueabi-)
set(CMAKE_C_COMPILER ${CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_PREFIX}g++)

include_directories(${PROJECT_SOURCE_DIR} ${CMAKE_SYSTEM_INCLUDE_PATH}/eigen3)

add_library(filter STATIC filter.cc integrate.cc)
add_library(mpu9250 STATIC mpu_9250.c)

add_executable(imu_measure main.cc mpu9250_unit.cc)
add_executable(set_bp set_bp.c)
add_executable(test_orientation test_orientation.cc)

add_dependencies(test_orientation filter)
add_dependencies(imu_measure mpu9250 filter)
add_dependencies(set_bp mpu9250)

target_link_libraries(test_orientation filter)
target_link_libraries(imu_measure mpu9250 filter)
target_link_libraries(set_bp mpu9250)

install(TARGETS imu_measure set_bp RUNTIME DESTINATION bin)
install(TARGETS mpu9250 filter ARCHIVE DESTINATION lib)
