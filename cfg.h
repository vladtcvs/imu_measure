#pragma once

#include <string>

struct control_config {
	float adjax, adjay, adjaz;
	float adjmx, adjmy, adjmz;
	float E2, F2;
	float G2x, G2y, G2z;
	float N2x, N2y, N2z;
	float rigidity_roll, rigidity_pitch, rigidity_yaw;
	int measures_start;
	int engines_cs;
	std::string spidev;
};

bool read_config(const std::string &filename);
bool read_parse_config(const std::string& file, control_config *cfg);
