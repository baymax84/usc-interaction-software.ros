#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"ik_cmd");
  ros::NodeHandle n;
  ros::Publisher joint_publisher = n.advertise<geometry_msgs::TransformStamped>("target_frame",1000);

  geometry_msgs::TransformStamped t;

  t.header.stamp = ros::Time::now();
  t.header.frame_id = "l_shoulder_pan_link";

/*
  t.transform.translation.x = 0.207;
  t.transform.translation.y = 0.131;
  t.transform.translation.z = -0.185;
 
  t.transform.rotation.x = 0.093;
  t.transform.rotation.y = 0.575;
  t.transform.rotation.z = 0.812;
  t.transform.rotation.w = -0.033;
*/
  t.transform.translation.x = 0.782;
  t.transform.translation.y = 0.187;
  t.transform.translation.z = 0.151;
 
  t.transform.rotation.x = 0.003;
  t.transform.rotation.y = -0.605;
  t.transform.rotation.z = -0.002;
  t.transform.rotation.w = 0.796;

  while( ros::ok() )
  {
    joint_publisher.publish(t);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  return 0;
}


