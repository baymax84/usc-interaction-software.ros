#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <wait_for_response_action/WaitForResponseAction.h>

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "test_response_server" );

	actionlib::SimpleActionClient<wait_for_response_action::WaitForResponseAction> ac("wait_for_response_server", true );
	ac.waitForServer();

	ROS_INFO( "action server started, sending goal" );

	wait_for_response_action::WaitForResponseGoal goal;
	ac.sendGoal(goal);

	bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

	if( finished_before_timeout )
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO( "action finished" );
	}
	else
		ROS_INFO( "Action did not finish before timeout" );
	
	return 0;
}

