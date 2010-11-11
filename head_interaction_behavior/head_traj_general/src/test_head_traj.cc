#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "test_traj" );
	ros::NodeHandle nh;
	ros::Publisher traj_pub = nh.advertise<geometry_msgs::PointStamped>("head_goal",1000);
	ros::Rate loop_rate(1);	

	geometry_msgs::PointStamped goal;
	goal.header.frame_id = "/child/base_link";
	goal.point.x = 0;
	goal.point.y = 0;
	goal.point.z = 0;

	while( ros::ok() )
	{
		goal.header.stamp = ros::Time::now();
		traj_pub.publish(goal);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
