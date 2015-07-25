#include <mpu9250_unit.h>
#include <set_pwm.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <read_config.h>
#include <fstream>
#include <sstream>
#include <iostream>

void close_all()
{
	
}

static void signal_hdl(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		close_all();
		exit(0);
	}
}


int main(int argc, char **argv)
{
	double dzw = 0.05;
	double dw = 0.01;
	double dzm = 0.05;
	double dm = 0.01;
	double dza = 0.05;
	double da = 0.01;
	double mass;
	int noff;
	Vector3d I;
	Vector3d w;
	Matrix3d R;
	Vector3d M, F;
	Vector3d v;
	char fname[256] = "copter.conf";
	char opts[]="c:";
	int opt;

	while ((opt = getopt(argc, argv, opts)) != -1) {
		switch (opt) {
		case 'c':
			strncpy(fname, optarg, 256);
			fname[255] = 0;
			break;
		default:
			break;
		}
	}
	std::cout<<"Using config file: "<<fname<<std::endl;
	std::ifstream config;
	config.open(fname);
	if (!config) {
		std::cout<<"config file not found!\n";
		exit(0);
	}
	std::stringstream ss;
	ss << config.rdbuf();
	config.close();
	std::map<std::string, std::string> options = parse_config(ss.str());
	std::map<std::string, std::string>::iterator it;
	std::cout<<"Options:\n";
	for (it = options.begin(); it != options.end(); ++it) {
		std::pair<std::string, std::string> opt = *it;
		std::cout<<"\t"<<opt.first<<" = "<<opt.second<<std::endl;
	}

	try {
		mass = parse_double(options["mass"]);
	} catch (int err) {
		std::cout<<"error parsing mass\n";
		mass = 1;
	}

	try {
		dzw = parse_double(options["dzw"]);
	} catch (int err) {
		std::cout<<"error parsing dzw\n";
		dzw = 1;
	}

	try {
		dw = parse_double(options["dw"]);
	} catch (int err) {
		std::cout<<"error parsing dw\n";
		dw = 1;
	}

	try {
		dza = parse_double(options["dza"]);
	} catch (int err) {
		std::cout<<"error parsing dza\n";
		dza = 1;
	}

	try {
		da = parse_double(options["da"]);
	} catch (int err) {
		std::cout<<"error parsing da\n";
		da = 1;
	}

	try {
		I = parse_vector3(options["I"]);
	} catch (int err) {
		std::cout<<"error parsing I\n";
		I.setOnes();
	}

	try {
		noff = parse_double(options["noff"]);
	} catch (int err) {
		std::cout<<"error parsing noff\n";
		noff = 20;
	}

	w.setZero();
	v.setZero();
	R.setZero();
	for (int i = 0; i < 3; i++)
	for (int j = 0; j < 3; j++)
		R(i, j) = (i == j);

	mpu9250_unit *imu;
	try {
		 imu = new mpu9250_unit("/dev/i2c-2", mass, I, w, R, v);
		 imu->measure_offset(noff);
		 delete imu;
	} catch(int code) {
		std::cout<<"Error opening mpu9250: ";
		switch (code) {
		case 1:
			std::cout<<"/dev/i2c-2 not found\n";
			break;
		default:
			std::cout<<"unknown\n";
			break;
		}
	}
	return 0;
}
