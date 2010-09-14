#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"ik_cmd");
  ros::NodeHandle n;
  ros::Publisher joint_publisher = n.advertise<geometry_msgs::PoseStamped>("target_frame",1000);

  geometry_msgs::PoseStamped t;

  t.header.stamp = ros::Time::now();
  t.header.frame_id = "bandit_torso_link";


  //  Reachable position (goto_cart test)
//  t.pose.position.x = 0.257;
//  t.pose.position.y = 0.232;
//  t.pose.position.z = 0.082;

  //  Unreachable position (point_to_cart test)
  t.pose.position.x = 0.5;
  t.pose.position.y = 0.5;
  t.pose.position.z = 0.5;
 
  t.pose.orientation.x = -0.404;
  t.pose.orientation.y = 0.417;
  t.pose.orientation.z = 0.807;
  t.pose.orientation.w = 0.108;

/*
  t.transform.translation.x = 0.782;
  t.transform.translation.y = 0.187;
  t.transform.translation.z = 0.151;
 
  t.transform.rotation.x = 0.003;
  t.transform.rotation.y = -0.605;
  t.transform.rotation.z = -0.002;
  t.transform.rotation.w = 0.796;
*/

  while( ros::ok() )
  {
    joint_publisher.publish(t);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  return 0;
}


