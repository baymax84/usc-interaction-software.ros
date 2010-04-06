#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

#include <geometry_msgs/PoseStamped.h>

navfn::NavfnROS navfn_plan;
ros::Subscriber goal_sub;

void goal_cb( const geometry_msgs::PoseStampedConstPtr &msg )
{
	geometry_msgs::PoseStamped start = *msg;



	start.pose.position.x = 1.0;	
	start.pose.position.y = -1.0;

	ROS_INFO( "plan: (%f,%f) ==> (%f,%f)", start.pose.position.x,start.pose.position.y,msg->pose.position.x,msg->pose.position.y );

	std::vector< geometry_msgs::PoseStamped > path;

	bool planned = navfn_plan.makePlan( start, *msg, path );
	if( !planned )
	{
		ROS_WARN( "plan did not succeed" );
	}
/*
	for( unsigned int i = 0; i < path.size(); i++ )
	{
		ROS_INFO( "path(%d): (%f,%f)", i, path[i].pose.position.x, path[i].pose.position.y );
	}
*/
	navfn_plan.publishPlan( path, 0.0, .8, 0.0, 0.2 );
}

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "overhead_planner" );
	ros::NodeHandle nh("~");	

	tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS costmap("ovh_costmap", tf );
	navfn_plan.initialize("ovh_navfn_planner", &costmap );

	goal_sub = nh.subscribe("/goal", 1, goal_cb );

	ros::spin();
	return 0;
}
