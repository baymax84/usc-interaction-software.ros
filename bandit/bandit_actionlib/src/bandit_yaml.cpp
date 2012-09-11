#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <yaml-cpp/yaml.h>

using namespace std;

struct Gestures {
	float id, joint_angle;
};

struct Frame {
	int frame_num;
	std::vector <Gestures> gestures;
};

void operator >> (const YAML::Node& node, Gestures& gesture){
	node["Joint ID"] >> gesture.id;
	node["Joint Angle"] >> gesture.joint_angle;
}

void operator >> (const YAML::Node& node, Frame& frame){
	node["Frame Num"] >> frame.frame_num;
	const YAML::Node& gestures = node["Gestures"];
	for (unsigned i=0; i<gestures.size();i++){
		Gestures gesture;
		gestures[i] >> gesture;
		frame.gestures.push_back(gesture);
	}
}

int main( int argc,  char** argv )
{
	ros::init(argc,argv,"bandit_yaml");
	std::ifstream fin;
	string filename;
	std::cout << "Input filename: ";
	std::cin >> filename;
	const std::string fileName = filename;
	fin.open(fileName.c_str());
	if (fin.fail()){
		ROS_WARN("Failure to find File");
	}
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	for(unsigned k=0;k < doc.size();k++) {
		Frame frames;
		doc[k] >> frames;
		std::cout << " frame num: " << frames.frame_num << "\n";
		for (int l =0; l < frames.gestures.size(); l++){
			Gestures g = frames.gestures[l];
			cout << " id: " <<  g.id << endl;
			cout << " joint angle: " << g.joint_angle << endl;
		}
	}
	
	return 0;
	
}
