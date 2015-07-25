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
	return d;
}

Eigen::Vector3d parse_vector3(std::string value)
{
	Eigen::Vector3d vec;
	return vec;
}

Eigen::Matrix3d parse_matrix3(std::string value)
{
	Eigen::Matrix3d vec;
	return vec;
}
