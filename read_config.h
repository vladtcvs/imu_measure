#ifndef __READCONFIG_H__
#define __READCONFIG_H__

#include <stdio.h>
#include <map>
#include <string>
#include <Eigen/Dense>

std::map<std::string, std::string> parse_config(std::string cfg);

int parse_int(std::string value);
double parse_double(std::string value);
Eigen::Vector3d parse_vector3(std::string value);
Eigen::Matrix3d parse_matrix3(std::string value);

#endif
