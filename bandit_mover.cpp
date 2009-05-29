#include "ros/ros.h"
#include "bandit_msgs/JointArray.h"

#define DTOR( a ) a * M_PI / 180.0

int main( int argc, char* argv[] )
{
  ros::init(argc,argv,"bandit_mover");
  ros::NodeHandle n;
  
  ros::Publisher joint = n.advertise<bandit_msgs::Joint>("joint_ind",1000);
  ros::Rate loop_rate(10);

  bandit_msgs::Joint j;
  j.id = 5;
  j.angle = 0;

  while( n.ok() )
  {
    j.angle = j.angle + DTOR(1);
    if( j.angle > DTOR(30) ) j.angle = 0;
    ROS_INFO( "setting joint [%d] to angle [%0.2f]", j.id, j.angle );

    joint.publish( j );

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
