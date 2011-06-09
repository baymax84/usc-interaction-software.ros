#include "ros/ros.h"
#include <joint_msgs/Params.h>
#include <joint_msgs/JointArray.h>
#include <sensor_msgs/JointState.h>
joint_msgs::Params::Response param_res;
#define DTOR( a ) (a * M_PI  / 180.0)
#define RTOD( a ) (a * 180.0 / M_PI)

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
  static int    id    = 0;
  static double angle = 0.0l;
  for ( int i = 0, n = j->joints.size(); i < n; ++i )
  {
    id            = j->joints[i].id;
    angle         = j->joints[i].angle;
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
  joint_state_publisher = n.advertise<sensor_msgs::JointState>(
    "fake_sparky", 1000);


  // angles in degrees
  add_param( 0, "Mouth",      -40,  0, 0);
  add_param( 1, "HeadNod",    -29, 45, 0);
  add_param( 2, "HeadTurn",   -30, 30, 0);
  add_param( 3, "RtArmForward", 0, 80, 0);
  add_param( 4, "RtArmOut",     0, 75, 0);
  add_param( 5, "RtElbow",      0, 90, 0);
  add_param( 6, "LtArmForward", 0, 80, 0);  
  add_param( 7, "LtArmOut",     0, 72, 0);
  add_param( 8, "LtElbow",      0, 90, 0);  
  add_param( 9, "RtWrist",    -45, 60, 0); 
  add_param(10, "LtWrist",    -35, 58, 0);  
  add_param(11, "TorsoBend",  -72,  0, 0);  
  add_param(12, "EyesBlink", -151,  0, 0);  
  add_param(13, "EyesLtRt",   -30, 22, 0);

  // distance in centimeters
  add_param(14, "RtFootUp",       3.25, 9.00, 0.00);
  add_param(15, "RtFootForward", -6.00, 6.00, 0.00);
  add_param(16, "LtFootUp",       3.50, 9.00, 0.00);
  add_param(17, "LtFootForward", -8.00, 4.00, 0.00);

  ROS_INFO("Advertising service 'sparky_params'...");
  ros::ServiceServer service = n.advertiseService("sparky_params", param);
  ROS_INFO("Service 'sparky_params' advertised!");
  ros::Rate loop_rate(50);
  
  sensor_msgs::JointState js;

  while( n.ok() )
  {
    js.header.frame_id = "/world";
    js.header.stamp    = ros::Time::now();
    js.name            = joint_names;
    js.position        = joint_pos;
    js.velocity        = zeroes;
    js.effort          = zeroes;

    joint_state_publisher.publish(js);
  
    //ROS_INFO( "publish" );

    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("I'm spun out...");

  return 0;
} // main(int, char* [])

