#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pose_action/PoseAction.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "test_pose_server" );

	actionlib::SimpleActionClient<pose_action::PoseAction> ac("pose_action_server", true );
	ac.waitForServer();

	ROS_INFO( "action server started, sending goal" );

	pose_action::PoseGoal goal;
	goal.duration = ros::Duration(3.0);
	ac.sendGoal(goal);

	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if( finished_before_timeout )
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO( "action finished" );
	}
	else
		ROS_INFO( "Action did not finish before timeout" );
	
	return 0;
}

