#include "mex.h"
#include <iostream>
#include <vector>
#include <ros/shared_memory_publisher.h>
#include <ros/shared_memory_subscriber.h>
#include <std_msgs/String.h>
#include <test_matlab/Vector3.h>
//#include <geometry_msgs/Twist.h>

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
	ros::SharedMemoryPublisher pub;
	ros::SharedMemorySubscriber sub( pub.getStorage().getName() );

	//std_msgs::String s;
	//s.data = "blah blah blah";

	test_matlab::Vector3 v;
	v.x = 5;
	v.y = 6;
	v.z = 7;

	//geometry_msgs::Twist t;
	//t.linear.x = 50;

	//std::string s( "hello!!" );
	//pub.publish( s.c_str(), s.size() );
	//pub.publishMessage( t );
	//pub.publishMessage( s );
	pub.publish( v );

	std::cout << "calling process" << std::endl;
	std::string command = "rosrun test_matlab_ros test_ros " + pub.getStorage().getName();
	std::system( command.c_str() );

	v = sub.fetch<test_matlab::Vector3>();

	std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
}
