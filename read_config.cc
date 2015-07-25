#include <read_config.h>
#include <string.h>

static bool isname(char c)
{
	return isalnum(c) || (c == '_');
}

static bool isval(char c)
{
	return isalnum(c) || (c == '_') || (c == ' ') || (c == '.');
}


std::map<std::string, std::string> parse_config(std::string cfg)
{
	std::map<std::string, std::string> cmap;
	int pnt = 0, len = cfg.length();

	while (pnt < len) {
		while (pnt < len && cfg[pnt] == ' ')
			pnt++;
		std::string name, val;
		while (pnt < len && isname(cfg[pnt])) {
			name += cfg[pnt];
			pnt++;
		}

		while (pnt < len && cfg[pnt] == ' ')
			pnt++;

		if (pnt < len && cfg[pnt] == '=') {
			pnt++;
			while (pnt < len && cfg[pnt] == ' ')
				pnt++;
			while (pnt < len && isval(cfg[pnt])) {
				val += cfg[pnt];
				pnt++;
			}
		}
		while (pnt < len && cfg[pnt] != '\n' && cfg[pnt] != '\r')
			pnt++;
		while (pnt < len && (cfg[pnt] == '\n' || cfg[pnt] == '\r'))
			pnt++;
		cmap.insert(std::pair<std::string, std::string>(name, val));
	}
	return cmap;
}

double parse_double(std::string value)
{
	double d = 0;
	std::istringstream i(value);
	if (!(i >> d))
		throw 1;
	return d;
}

Eigen::Vector3d parse_vector3(std::string value)
{
	Eigen::Vector3d vec;
	int j;
	std::istringstream i(value);
	for (j = 0; j < 3; j++) {
		double d;
		if (!(i >> d))
			throw 1;
		vec(j) = d;
	}
	return vec;
}

Eigen::Matrix3d parse_matrix3(std::string value)
{
	int j, k;
	Eigen::Matrix3d mat;
	std::istringstream i(value);
	for (j = 0; j < 3; j++)
	for (k = 0; k < 3; k++) {
		double d;
		if (!(i >> d))
			throw 1;
		mat(j, k) = d;
	}
	return mat;
}
