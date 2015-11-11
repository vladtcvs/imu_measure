cmake_minimum_required(VERSION 2.6)
project(mpu_9250)

add_definitions(-DDEBUG_GYRO=0)
#set(CMAKE_SYSTEM_INCLUDE_PATH /usr/include/)
set(CMAKE_SYSTEM_INCLUDE_PATH /home/vlad/module/buildroot/buildroot/output/host/usr/arm-buildroot-linux-gnueabi/sysroot/usr/include)
set(TOOCHAIN_BIN /home/vlad/module/buildroot/buildroot/output/host/usr/bin)
set(CROSS_PREFIX ${TOOCHAIN_BIN}/arm-buildroot-linux-gnueabi-)
set(CMAKE_C_COMPILER ${CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_PREFIX}g++)

set (CMAKE_CXX_FLAGS "-g")
set (CMAKE_C_FLAGS "-g")

include_directories(${PROJECT_SOURCE_DIR} ${CMAKE_SYSTEM_INCLUDE_PATH}/eigen3)

add_library(engines_set STATIC set_pwm.cc)
add_library(mpu9250 STATIC mpu_9250.c)

add_executable(control main.cc mpu9250_unit.cc integrate.cc stabilization.cc)
add_executable(set_bp set_bp.c)

target_link_libraries(control mpu9250 engines_set)
target_link_libraries(set_bp mpu9250)
