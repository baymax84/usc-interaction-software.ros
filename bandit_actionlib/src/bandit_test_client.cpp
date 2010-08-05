#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <bandit_actionlib/BanditAction.h>
#include <sensor_msgs/JointState.h>
#include <bandit_msgs/JointArray.h>
#include <bandit_msgs/Params.h>
#include <yaml-cpp/yaml.h>

using namespace std;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_bandit_action");
	//create the action client
	//true will cause the client to spin its own thread
	actionlib::SimpleActionClient<bandit_actionlib::BanditAction> ac("bandit_action", true);
  
	ROS_INFO("Waiting for Action Server");
	ac.waitForServer();
	ROS_INFO("Action Server Started, Sending Goal");
  
	string filename;
	std::cout << "Please input gesture file name: ";
	std::cin >> filename;
  
	bandit_actionlib::BanditGoal goal;
	goal.frame = filename;
	ac.sendGoal(goal);
	
	bool fin_before_timeout = ac.waitForResult(ros::Duration(30.0));//tells the client to wait 30.0 seconds before it declares timeout
	
	if (fin_before_timeout)//the server is not responding and the console outputs a timeout message
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

  return 0;
	
}
