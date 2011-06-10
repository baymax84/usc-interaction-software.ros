#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

double threshold_;

void joint_cb( const sensor_msgs::JointStateConstPtr &vel )
{
	bool moving = false;

	for( int i = 0; i < vel->velocity.size(); i++ )
	{
		if( vel->velocity[i] > threshold_ )
		{
			moving  = true;
			break;
		}
	}

	ROS_INFO( "moving: %d", moving );
}

int main( int argc, char* argv[] )
{

	ros::init( argc, argv, "currently_posing" );
	ros::NodeHandle nh;
	nh.param( "threshold", threshold_, 0.03 );
	ros::Subscriber joint_sub = nh.subscribe( "joints_with_vel", 1, joint_cb );

	ros::spin();

	return 0;
}
