#include <ros/ros.h>
#include <deque>
#include <sensor_msgs/JointState.h>

ros::Publisher joints_pub;

std::deque<sensor_msgs::JointState> cache;

void joint_cb( const sensor_msgs::JointStateConstPtr &jnt )
{
	cache.push_back(*jnt);
	while( cache.size() > 2 ) cache.pop_front();

	sensor_msgs::JointState ret_jnt = *jnt;
	for( int i = 0; i < jnt->velocity.size(); i++ )
	{
		ret_jnt.velocity[i] = cache.front().position[i]-cache.back().position[i];
	}

	joints_pub.publish( ret_jnt );
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "joint_velocities" );
	ros::NodeHandle nh;

	joints_pub = nh.advertise<sensor_msgs::JointState>("joints_with_vel",1);
	ros::Subscriber joints_sub = nh.subscribe("target_joints", 1, &joint_cb);

	ros::spin();

	return 0;	
}

