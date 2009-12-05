#include "ros/ros.h"
#include <bandit_msgs/Params.h>
#include <bandit_msgs/Joint.h>
#include <sensor_msgs/JointState.h>
bandit_msgs::Params::Response param_res;
#define DTOR( a ) a * M_PI / 180.0
#define RTOD( a ) a * 180.0 / M_PI

ros::Publisher joint_state_publisher;
std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> zeroes;

bool param(bandit_msgs::Params::Request    &req,
           bandit_msgs::Params::Response   &res )
{
  res = param_res;
  return true;
}

void add_param( int id, std::string name, float min, float max, float pos )
{
  param_res.id.push_back(id);
	param_res.name.push_back(name);
	param_res.min.push_back(min);
	param_res.max.push_back(max);
	param_res.pos.push_back(pos);
  joint_pos.push_back(pos);
  joint_names.push_back(name);
  zeroes.push_back(0);
}

void jointCB( const bandit_msgs::JointConstPtr& j )
{
  joint_pos[j->id] = j->angle;
}


int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "fake_bandit" );
	ros::NodeHandle n;
  ros::Subscriber joint_sub = n.subscribe("joint_ind", 1, jointCB );
  joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);

  
  add_param( 0, "bandit_head_pan_joint", -90, 90, 0 );
  add_param( 1, "bandit_head_tilt_joint", -15, 15, 0 );
  add_param( 2, "bandit_luarm_tilt_joint", -90, 90, 0 );

	ros::ServiceServer service = n.advertiseService("params", param);
  ros::Rate loop_rate(1);
  
  sensor_msgs::JointState js;

  while( n.ok() )
  {
    js.header.frame_id="/world";
    js.name = joint_names;
    js.position = joint_pos;
    js.velocity = zeroes;
    js.effort = zeroes;    
    joint_state_publisher.publish(js);

	  ros::spinOnce();
    loop_rate.sleep();
  }

	return 0;
}
