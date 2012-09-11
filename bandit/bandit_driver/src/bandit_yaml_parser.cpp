#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <sstream>
#include <yaml-cpp/yaml.h>

using namespace std;

struct Joint_Calibrations {
	float id, truezero, offset, maxAngle, minAngle;
};

void operator >> (const YAML::Node& node, Joint_Calibrations& joint){
	node["Joint ID"] >> joint.id;
	node["True Zero"] >> joint.truezero;
	node["Offset"] >> joint.offset;
	node["Max Angle"] >> joint.maxAngle;
	node["Min Angle"] >> joint.minAngle;
}

int main( int argc, char* argv[] )
{
	ros::init(argc,argv,"bandit_yaml_parser");
	const std::string fileName = "Bandit_Calibration_File.yaml";
	std::ifstream fin;
	fin.open(fileName.c_str());
	if (fin.fail()){
		ROS_WARN("Failure to find File");
	}
	
	YAML::Node doc;
	YAML:: Parser parser(fin);	
	parser.GetNextDocument(doc);
	
	int j_id;
	double j_cal[19];
	while(ros::ok()){
		std::cout << "Please Input Desired Joint Number:";
		std::cin >> j_id;
	
		for(unsigned i=0;i<doc.size();i++) {
			Joint_Calibrations joint;
			doc[i] >> joint;
			if (j_id == joint.id){
				std::cout << joint.id << "\n";
				std::cout << joint.truezero << "\n";
				std::cout << joint.offset << "\n";
				std::cout << joint.maxAngle << "\n";
				std::cout << joint.minAngle << "\n";
				std::cout << "\n";
			}
		}
	}
	
	return 0;
}
