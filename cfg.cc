#include <cfg.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include <map>
#include <vector>

std::map<std::string, std::string> config;

int read_cfg_int(const std::string& value)
{
	int v;
	if (sscanf(value.c_str(), "%d", &v) <= 0)
		throw "can not parse int";
	return v;
}

float read_cfg_float(const std::string& value)
{
	float v;
	if (sscanf(value.c_str(), "%f", &v) <= 0)
		throw "can not parse float";
	return v;
}

std::string read_cfg_string(const std::string& value)
{
	int i;
	std::string re;
	i = 0;
	while (value[i] == ' ')
		i++;
	while (value[i] && value[i] != '\r' && value[i] != '\n')
		re += value[i++];
	return re;
}

std::vector<float> read_cfg_vector_float(const std::string& value)
{
	std::vector<float> v;
	int i = 0, j;
	int len = value.length();
	while (i < len && value[i] == ' ')
		i++;
	if (i == len || value[i] != '{')
		throw "can not parse vector";
	i++;
	if (i == len)
		throw "can not parse vector";

	do {
		if (isdigit(value[i]) || value[i] == '.' || value[i] == '-') {
			j = i;
			const char *p = value.c_str() + i;
			while (isdigit(value[j]) || value[j] == '.' || value[j] == '-')
				j++;
			std::string item = std::string(p, j - i);
			float val = read_cfg_float(item);
			v.push_back(val);
			i = j;
		} else if (value[i] == ' ') {
			i++;
		} else if (value[i] == '}') {
			break;
		} else {
			throw "can not parse vector";
		}
	} while (i < len);
	
	return v;
}

bool read_config(const std::string &filename)
{
	bool ret = false;
	FILE *F;
	F = fopen(filename.c_str(), "rt");
	if (F == NULL) {
		printf("Can not find file %s\n", filename.c_str());
		return false;
	}

	while (!feof(F)) {
		char buf[1024];
		if (fgets(buf, 1024, F) == NULL)
			continue;
		// TODO: check if strlen == 1023
		char *p1 = buf;
		while (*p1 == ' ')
			p1++;
		if (!*p1)
			continue;
		if (!isalnum(*p1))
			goto on_error;

		char *p2 = p1;
		while (isalnum(*p2) || *p2 == '_')
			p2++;
		if (!(*p2 == ' ' || *p2 == '='))
			goto on_error;

		std::string name = std::string(p1, p2 - p1);
		p1 = p2;
		int eq = 0;
		while (*p1 && *p1 == ' ' || *p1 == '=') {
			if (*p1 == '=')
				eq++;
			p1++;
		}
		if (eq != 1)
			goto on_error;

		p2 = p1;
		while (*p2 && *p2 != '#')
			p2++;
		std::string value(p1, p2 - p1);
		config.insert(std::make_pair(name, value));
	}
	ret = true;
on_error:
	fclose(F);
	return ret;
}

bool read_parse_config(const std::string& file, control_config *cfg)
{
	if (read_config(file) == false)
		return false;
	try {
		std::vector<float> vec;
		cfg->E2 = read_cfg_float(config["E2"]);
		cfg->F2 = read_cfg_float(config["F2"]);
		cfg->measures_start = read_cfg_int(config["measures_at_start"]);
		cfg->engines_cs = read_cfg_int(config["engines_cs"]);
		cfg->spidev = read_cfg_string(config["spidev"]);
		vec = read_cfg_vector_float(config["adja"]);
		if (vec.size() != 3)
			throw "adja vector has wrong size\n";
		cfg->adjax = vec[0];
		cfg->adjay = vec[1];
		cfg->adjaz = vec[2];
		vec = read_cfg_vector_float(config["adjm"]);
		if (vec.size() != 3)
			throw "adjm vector has wrong size\n";
		cfg->adjmx = vec[0];
		cfg->adjmy = vec[1];
		cfg->adjmz = vec[2];
		vec = read_cfg_vector_float(config["rigidity"]);
		if (vec.size() != 3)
			throw "rigidity vector has wrong size\n";
		cfg->rigidity_roll = vec[0];
		cfg->rigidity_pitch = vec[1];
		cfg->rigidity_yaw = vec[2];
	} catch (std::string err) {
		printf("Error parsing: %s\n", err.c_str());
		return false;
	} catch (...) {
		printf("Error\n");
		return false;
	}
	printf("Config:\n");
	printf("\tadja = %f %f %f\n", cfg->adjax, cfg->adjay, cfg->adjaz);
	printf("\tadjm = %f %f %f\n", cfg->adjmx, cfg->adjmy, cfg->adjmz);
	printf("\trigidity = %f %f %f\n", cfg->rigidity_roll, cfg->rigidity_pitch, cfg->rigidity_yaw);
	printf("\tnumber of measurements at start= %d\n", cfg->measures_start);
	printf("\tE2 = %f F2 = %f\n", cfg->E2, cfg->F2);
	printf("\tengines cs = %d\n", cfg->engines_cs);
	printf("\tspidev = [%s]\n", cfg->spidev.c_str());
	return true;
}
