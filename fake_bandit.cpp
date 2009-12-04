#include "ros/ros.h"

#include <bandit_msgs/Params.h>

bandit_msgs::Params::Response param_res;
#define DTOR( a ) a * M_PI / 180.0
#define RTOD( a ) a * 180.0 / M_PI



bool param(bandit_msgs::Params::Request    &req,
           bandit_msgs::Params::Response   &res )
{
  res = param_res;
  return true;
}


int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "fake_bandit" );
	ros::NodeHandle n;

	param_res.id.push_back(0);
	param_res.name.push_back("pan");
	param_res.min.push_back(-90);
	param_res.max.push_back( 90);
	param_res.pos.push_back(  0);

	param_res.id.push_back(1);
	param_res.name.push_back("tilt");
	param_res.min.push_back(-90);
	param_res.max.push_back( 90);
	param_res.pos.push_back(  0);


	ros::ServiceServer service = n.advertiseService("params", param);

	ros::spin();

	return 0;
}
