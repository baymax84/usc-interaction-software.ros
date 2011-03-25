#include "ros/ros.h"
#include <joint_msgs/Params.h>
#include <joint_msgs/JointArray.h>
#include <sensor_msgs/JointState.h>
joint_msgs::Params::Response param_res;
#define DTOR( a ) a * M_PI / 180.0
#define RTOD( a ) a * 180.0 / M_PI

ros::Publisher joint_state_publisher;
std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> zeroes;

bool param(joint_msgs::Params::Request    &req,
           joint_msgs::Params::Response   &res )
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
	param_res.curr.push_back(pos);
  joint_pos.push_back(pos);
  joint_names.push_back(name);
  zeroes.push_back(0);
}

void jointCB( const joint_msgs::JointConstPtr& j )
{
  joint_pos[j->id] = j->angle;
}

void jointarrayCB( const joint_msgs::JointArrayConstPtr& j )
{
	for( int i = 0; i < j->joints.size(); i++ )
	{
		int id = j->joints[i].id;
		double angle = j->joints[i].angle;
		joint_pos[id] = angle;
	}
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "fake_sparky" );
	ros::NodeHandle n;
  ros::Subscriber joint_sub = n.subscribe("joint_cmd", 1, jointCB );
  //ros::Subscriber joint_sub = n.subscribe("joint_cmd", 1, jointCB );
  ros::Subscriber joints_sub = n.subscribe("joints", 1, jointarrayCB );
  joint_state_publisher = n.advertise<sensor_msgs::JointState>("fake_sparky",1000);



  add_param( 0, "Mouth", -42, 0, 0 );
  add_param( 1, "HeadNod", -1, 60, 0 );
  add_param( 2, "HeadTurn", -25, 45, 0 );
  add_param( 3, "RtArmForward", 2, 91, 0);
  add_param( 4,  "RtArmOut", 3, 72, 0);
  add_param( 5, "RtElbow", 13, 104, 0);
  add_param( 6, "LtArmForward", 3, 90, 0);  
  add_param( 7, "LtArmOut", 2, 72, 0);
  add_param( 8, "LtElbow", 15, 105, 0);  
  add_param( 9, "RtWrist", -45, 90, 0); 
  add_param( 10, "LtWrist", -42, 90, 0);  
  add_param( 11, "TorsoBend", 1.3, 75.8, 0);  
  add_param( 12, "EyesBlink", 0, 102, 0);  
  add_param( 13, "EyesLtRt", -30, 40, 0);
  add_param( 14, "RtFootUp", 0, 4, 0);
  add_param( 15, "RtFootForward", -10, 10, 0);
  add_param( 16, "LtFootUp", 0, 4, 0);
  add_param( 17, "LtFootForward", -10, 10, 0);

  


  ROS_INFO("Advertising service 'sparky_params'...");
	ros::ServiceServer service = n.advertiseService("sparky_params", param);
  ROS_INFO("Service 'sparky_params' advertised!");
  ros::Rate loop_rate(50);
  
  sensor_msgs::JointState js;

  while( n.ok() )
  {
    js.header.frame_id="/world";
    js.header.stamp = ros::Time::now();
    js.name = joint_names;
    js.position = joint_pos;
    js.velocity = zeroes;
    js.effort = zeroes;    



    joint_state_publisher.publish(js);
  
    //ROS_INFO( "publish" );

 
	  ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("I'm spun out...");

	return 0;
}
